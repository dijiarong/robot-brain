from __future__ import annotations

import asyncio
import time
import unittest

from fastapi.testclient import TestClient

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.actuation.unitree import FakeUnitreeTransport, UnitreeRobot
from robot_brain.core.world_state import DetectedObject
from robot_brain.perception.base import Observation
from robot_brain.perception.mock import MockPerception
from robot_brain.core.tasks import TaskStatus
from robot_brain.navigation.base import NavigationUnavailableError
from robot_brain.runtime.loop import AgentRuntime
from robot_brain.runtime.scheduler import AgentScheduler
from robot_brain.service.app import create_app
from robot_brain.service.runner import AgentService


class AgentServiceTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self) -> None:
        from robot_brain.control.authority import reset_motion_authority_for_tests

        reset_motion_authority_for_tests()

    async def test_service_owns_unitree_transport_connection(self) -> None:
        settings = Settings(
            robot_backend="unitree", unitree_transport="fake",
            unitree_dry_run=False, unitree_enable_motion=True,
            memory_db_path=":memory:",
        )
        transport = FakeUnitreeTransport()
        robot = UnitreeRobot(transport, settings)
        runtime = AgentRuntime.create(settings=settings, robot=robot)
        service = AgentService(
            AgentScheduler(runtime), poll_interval=0.01,
            close_runtime_on_stop=False,
        )

        await service.start()
        self.assertTrue(transport.is_connected)
        await service.stop()
        self.assertFalse(transport.is_connected)
        runtime.close()

    async def test_dashboard_keyboard_teleop_uses_deadman_session(self) -> None:
        settings = Settings(
            robot_backend="unitree", unitree_transport="fake",
            unitree_dry_run=False, unitree_enable_motion=True,
            memory_db_path=":memory:",
        )
        robot = UnitreeRobot(FakeUnitreeTransport(), settings)
        runtime = AgentRuntime.create(settings=settings, robot=robot)
        service = AgentService(
            AgentScheduler(runtime), poll_interval=0.01,
            close_runtime_on_stop=False,
        )

        await service.start()
        accepted = await service.set_web_teleop(0.2, 0.0, 0.0)
        stopped = await service.stop_web_teleop()
        await service.stop()

        self.assertTrue(accepted["accepted"])
        self.assertTrue(stopped["accepted"])
        runtime.close()

    async def test_service_start_does_not_auto_stand_on_fake_transport(self) -> None:
        settings = Settings(
            robot_backend="unitree", unitree_transport="fake",
            unitree_dry_run=False, unitree_enable_motion=True,
            memory_db_path=":memory:",
        )
        robot = UnitreeRobot(FakeUnitreeTransport(), settings)
        runtime = AgentRuntime.create(settings=settings, robot=robot)
        service = AgentService(
            AgentScheduler(runtime), poll_interval=0.01,
            close_runtime_on_stop=False,
        )

        await service.start()
        await service.stop()

        # Fake transport is not live webrtc/sdk, so no posture commands.
        self.assertFalse(any(entry["action"] == "set_posture" for entry in robot.action_history))
        runtime.close()

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
        original_aclose = runtime.aclose

        async def aclose() -> None:
            nonlocal closed
            closed = True
            await original_aclose()

        runtime.aclose = aclose  # type: ignore[method-assign]

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
        status_json = status.json()
        self.assertTrue(status_json["service"]["running"])
        # Iteration 18: VLM + explore diagnostics exposed.
        self.assertIn("vlm", status_json)
        self.assertFalse(status_json["vlm"]["enabled"])  # VLM off by default
        self.assertIn("explore", status_json)
        self.assertIn("last_stop_reason", status_json["explore"])

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

    def test_navigation_snapshot_is_read_only_and_structured(self) -> None:
        with self.client:
            response = self.client.get("/api/navigation/snapshot")

        self.assertEqual(200, response.status_code)
        payload = response.json()
        self.assertTrue(payload["available"])
        self.assertEqual("FakeNavigationClient", payload["provider"])
        self.assertIn("state", payload)
        self.assertIsNone(payload["costmap"])

    def test_map_goal_reports_stale_sensors_instead_of_failing(self) -> None:
        navigation = self.runtime.context.navigation

        async def stale_costmap():
            raise NavigationUnavailableError("stale_pointcloud")

        navigation.get_costmap = stale_costmap  # type: ignore[method-assign]
        with self.client:
            response = self.client.post("/api/navigation/map-goal", json={
                "x_m": 0.5, "y_m": 0.0, "confirm": "I_UNDERSTAND_MAP_NAVIGATION",
            })

        self.assertEqual(503, response.status_code)
        self.assertIn("stale_pointcloud", response.json()["detail"])

    def test_navigation_cancel_endpoint_is_idempotent(self) -> None:
        with self.client:
            response = self.client.post("/api/navigation/cancel")
        self.assertEqual(200, response.status_code)
        payload = response.json()
        self.assertIn("status", payload)

    def test_posture_endpoint_rejects_mock_backend(self) -> None:
        with self.client:
            response = self.client.post("/api/teleop/posture", json={"posture": "stand_up"})

        self.assertEqual(503, response.status_code)

    def test_posture_endpoint_issues_unitree_posture(self) -> None:
        settings = Settings(
            robot_backend="unitree", unitree_transport="fake",
            unitree_dry_run=True, unitree_enable_motion=True,
            memory_db_path=":memory:",
        )
        robot = UnitreeRobot(FakeUnitreeTransport(), settings)
        runtime = AgentRuntime.create(settings=settings, robot=robot)
        service = AgentService(
            AgentScheduler(runtime), poll_interval=60,
            close_runtime_on_stop=False,
        )
        client = TestClient(create_app(service))

        with client:
            response = client.post("/api/teleop/posture", json={"posture": "stand_up"})

        self.assertEqual(200, response.status_code)
        self.assertEqual("stand_up", response.json()["posture"])
        self.assertEqual("set_posture", robot.action_history[-1]["action"])
        runtime.close()


if __name__ == "__main__":
    unittest.main()
