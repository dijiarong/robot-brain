from __future__ import annotations

import asyncio
import time
import unittest

from fastapi.testclient import TestClient

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.world_state import DetectedObject
from robot_brain.perception.base import Observation
from robot_brain.perception.mock import MockPerception
from robot_brain.core.tasks import TaskStatus
from robot_brain.runtime.loop import AgentRuntime
from robot_brain.runtime.scheduler import AgentScheduler
from robot_brain.service.app import create_app
from robot_brain.service.runner import AgentService


class AgentServiceTests(unittest.IsolatedAsyncioTestCase):
    async def test_background_service_consumes_queued_task(self) -> None:
        robot = MockRobot()
        runtime = AgentRuntime.create(settings=Settings(memory_db_path=":memory:"), robot=robot)
        scheduler = AgentScheduler(runtime)
        service = AgentService(scheduler, poll_interval=0.01, close_runtime_on_stop=False)
        task = scheduler.submit("stop")

        await service.start()
        for _ in range(50):
            if scheduler.tasks.get(task.task_id).status == TaskStatus.COMPLETED:
                break
            await asyncio.sleep(0.01)
        await service.stop()

        self.assertEqual(TaskStatus.COMPLETED, scheduler.tasks.get(task.task_id).status)
        self.assertEqual("stop", robot.action_history[0]["action"])
        self.assertFalse(service.running)
        runtime.close()

    async def test_stop_closes_runtime(self) -> None:
        runtime = AgentRuntime.create(settings=Settings(memory_db_path=":memory:"))
        service = AgentService(AgentScheduler(runtime), poll_interval=0.01)
        closed = False
        original_close = runtime.close

        def close() -> None:
            nonlocal closed
            closed = True
            original_close()

        runtime.close = close  # type: ignore[method-assign]

        await service.start()
        await service.stop()

        self.assertTrue(closed)
        self.assertFalse(service.running)


class ServiceAPITests(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = MockRobot()
        self.runtime = AgentRuntime.create(settings=Settings(memory_db_path=":memory:"), robot=self.robot)
        self.service = AgentService(
            AgentScheduler(self.runtime),
            poll_interval=60,
            close_runtime_on_stop=False,
        )
        self.client = TestClient(create_app(self.service))

    def tearDown(self) -> None:
        self.runtime.close()

    def test_dashboard_health_and_status(self) -> None:
        with self.client:
            dashboard = self.client.get("/")
            health = self.client.get("/health")
            status = self.client.get("/api/status")

        self.assertEqual(200, dashboard.status_code)
        self.assertIn("Robot Brain", dashboard.text)
        self.assertEqual({"status": "ok", "service_running": True}, health.json())
        self.assertTrue(status.json()["service"]["running"])

    def test_task_create_query_and_cancel(self) -> None:
        with self.client:
            created = self.client.post("/api/tasks", json={"objective": "patrol the lobby", "priority": 7})
            task_id = created.json()["task_id"]
            fetched = self.client.get(f"/api/tasks/{task_id}")
            listed = self.client.get("/api/tasks")
            cancelled = self.client.delete(f"/api/tasks/{task_id}")

        self.assertEqual(201, created.status_code)
        self.assertEqual(task_id, fetched.json()["task_id"])
        self.assertEqual(1, len(listed.json()))
        self.assertEqual("cancelled", cancelled.json()["status"])

    def test_estop_and_reset(self) -> None:
        with self.client:
            stopped = self.client.post(
                "/api/events",
                json={"type": "interrupt", "message": "api emergency stop"},
            )
            active = self.client.get("/api/status")
            reset = self.client.post("/api/estop/reset")
            cleared = self.client.get("/api/status")

        self.assertEqual("interrupted", stopped.json()["status"])
        self.assertTrue(active.json()["world"]["estop_active"])
        self.assertEqual("reset", reset.json()["status"])
        self.assertFalse(cleared.json()["world"]["estop_active"])

    def test_confirmation_endpoint_resumes_task(self) -> None:
        robot = MockRobot()
        runtime = AgentRuntime.create(
            settings=Settings(memory_db_path=":memory:"),
            robot=robot,
            perception=MockPerception(
                robot,
                [Observation(detected_objects=[DetectedObject(object_id="person-1", kind="person")])],
            ),
        )
        service = AgentService(AgentScheduler(runtime), poll_interval=0.01, close_runtime_on_stop=False)
        client = TestClient(create_app(service))

        with client:
            created = client.post("/api/tasks", json={"objective": "follow person-1"})
            task_id = created.json()["task_id"]
            pending = None
            for _ in range(50):
                pending = client.get(f"/api/tasks/{task_id}").json()
                if pending["status"] == TaskStatus.AWAITING_CONFIRMATION:
                    break
                time.sleep(0.01)
            resumed = client.post(f"/api/tasks/{task_id}/confirm", json={"approved": True})

        self.assertEqual(TaskStatus.AWAITING_CONFIRMATION, pending["status"])
        self.assertEqual("completed", resumed.json()["task"]["status"])
        self.assertEqual("follow", robot.action_history[0]["action"])
        runtime.close()

    def test_websocket_receives_initial_status(self) -> None:
        with self.client:
            with self.client.websocket_connect("/ws") as websocket:
                message = websocket.receive_json()

        self.assertEqual("status", message["type"])
        self.assertTrue(message["status"]["service"]["running"])


if __name__ == "__main__":
    unittest.main()
