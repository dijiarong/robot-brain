"""Evaluation harness: runs fixed scenarios and asserts Planner behavior."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from config.settings import Settings
from robot_brain.actuation.mock import MockRobot
from robot_brain.core.world_state import DetectedObject
from robot_brain.llm.mock import MockLLM
from robot_brain.perception.base import Observation
from robot_brain.perception.mock import MockPerception
from robot_brain.runtime.loop import AgentRuntime, RunResult

from .scenarios import Scenario


@dataclass
class ScenarioResult:
    """Result of running a single evaluation scenario."""

    scenario_name: str
    passed: bool
    run_result: RunResult | None = None
    assertions: list[tuple[str, bool, str]] = field(default_factory=list)
    error: str | None = None


class EvaluationHarness:
    """Run scenarios against the runtime and assert expected behavior."""

    def __init__(self, settings_overrides: dict[str, Any] | None = None) -> None:
        self._base_settings = settings_overrides or {}
        self.results: list[ScenarioResult] = []

    async def run_scenario(self, scenario: Scenario) -> ScenarioResult:
        """Execute a single scenario and validate assertions."""
        result = ScenarioResult(scenario_name=scenario.name, passed=True)

        try:
            runtime = self._build_runtime(scenario)
            run_result = await runtime.run_command(scenario.command)
            result.run_result = run_result

            # Check expected status
            if scenario.expected_status is not None:
                passed = run_result.status == scenario.expected_status
                msg = f"Expected status={scenario.expected_status}, got={run_result.status}"
                result.assertions.append(("status", passed, msg))
                if not passed:
                    result.passed = False

            # Check expected decision source
            if scenario.expected_decision_source is not None:
                passed = run_result.decision_source == scenario.expected_decision_source
                msg = f"Expected source={scenario.expected_decision_source}, got={run_result.decision_source}"
                result.assertions.append(("decision_source", passed, msg))
                if not passed:
                    result.passed = False

            # Check expected skills
            if scenario.expected_skills is not None:
                actual_skills = [r.skill_name for r in run_result.results if hasattr(r, "skill_name")]
                passed = actual_skills == scenario.expected_skills
                msg = f"Expected skills={scenario.expected_skills}, got={actual_skills}"
                result.assertions.append(("skills", passed, msg))
                if not passed:
                    result.passed = False

            # Run custom assertion
            if scenario.custom_assert is not None:
                custom_passed, custom_msg = scenario.custom_assert(run_result)
                result.assertions.append(("custom", custom_passed, custom_msg))
                if not custom_passed:
                    result.passed = False

            runtime.close()

        except Exception as exc:
            result.passed = False
            result.error = str(exc)

        self.results.append(result)
        return result

    async def run_all(self, scenarios: list[Scenario]) -> list[ScenarioResult]:
        """Run all provided scenarios and return results."""
        results = []
        for scenario in scenarios:
            sr = await self.run_scenario(scenario)
            results.append(sr)
        return results

    def summary(self) -> dict[str, Any]:
        """Return a summary of all runs."""
        total = len(self.results)
        passed = sum(1 for r in self.results if r.passed)
        return {
            "total": total,
            "passed": passed,
            "failed": total - passed,
            "scenarios": [
                {
                    "name": r.scenario_name,
                    "passed": r.passed,
                    "assertions": r.assertions,
                    "error": r.error,
                }
                for r in self.results
            ],
        }

    def _build_runtime(self, scenario: Scenario) -> AgentRuntime:
        """Build a fresh runtime configured for this scenario."""
        settings_dict: dict[str, Any] = {"memory_db_path": ":memory:", **self._base_settings}
        settings_dict.update(scenario.settings_overrides)
        settings = Settings(**settings_dict)

        robot = MockRobot()

        # Build perception with scripted observations
        observations: list[Observation] = []
        if scenario.observations:
            for obs_dict in scenario.observations:
                # Convert detected_objects dicts to DetectedObject models if present
                if "detected_objects" in obs_dict:
                    obs_dict = obs_dict.copy()
                    obs_dict["detected_objects"] = [
                        DetectedObject(**obj) if isinstance(obj, dict) else obj
                        for obj in obs_dict["detected_objects"]
                    ]
                observations.append(Observation(**obs_dict))
        perception = MockPerception(robot, observations) if observations else MockPerception(robot)

        # Build LLM with scripted plans
        llm = MockLLM(scenario.scripted_plans) if scenario.scripted_plans else MockLLM()

        return AgentRuntime.create(
            settings=settings,
            robot=robot,
            perception=perception,
            llm=llm,
        )
