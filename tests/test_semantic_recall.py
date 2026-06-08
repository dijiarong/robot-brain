"""Tests for semantic experience recall using TF-IDF."""
from __future__ import annotations

import unittest

from robot_brain.memory.long_term import Experience, LongTermMemory
from robot_brain.memory.semantic_store import SemanticExperienceStore, TfidfIndex, _tokenize


class TokenizeTests(unittest.TestCase):
    def test_basic_tokenize(self) -> None:
        tokens = _tokenize("Hello World! This is a test.")
        self.assertEqual(["hello", "world", "this", "is", "a", "test"], tokens)

    def test_chinese_tokenize(self) -> None:
        tokens = _tokenize("巡逻 patrol 任务")
        self.assertIn("巡逻", tokens)
        self.assertIn("patrol", tokens)
        self.assertIn("任务", tokens)

    def test_empty_string(self) -> None:
        self.assertEqual([], _tokenize(""))

    def test_numbers(self) -> None:
        tokens = _tokenize("move to x=3.5 y=7")
        self.assertIn("3", tokens)
        self.assertIn("5", tokens)
        self.assertIn("7", tokens)


class TfidfIndexTests(unittest.TestCase):
    def test_empty_search(self) -> None:
        index = TfidfIndex()
        self.assertEqual([], index.search(["hello"]))

    def test_single_document(self) -> None:
        index = TfidfIndex()
        index.add(["patrol", "the", "lobby"])
        results = index.search(["patrol"])
        self.assertEqual(1, len(results))
        self.assertEqual(0, results[0][0])
        self.assertGreater(results[0][1], 0)

    def test_relevance_ranking(self) -> None:
        index = TfidfIndex()
        index.add(["navigate", "to", "kitchen"])  # doc 0
        index.add(["patrol", "lobby", "hallway"])  # doc 1
        index.add(["patrol", "perimeter", "fence"])  # doc 2

        results = index.search(["patrol", "lobby"])
        # Doc 1 should rank highest (has both "patrol" and "lobby")
        self.assertEqual(1, results[0][0])


class SemanticExperienceStoreTests(unittest.TestCase):
    def test_add_and_search(self) -> None:
        store = SemanticExperienceStore()
        store.add(Experience(
            objective="patrol the lobby",
            outcome="completed",
            summary="Successfully patrolled lobby area, no threats detected",
        ))
        store.add(Experience(
            objective="navigate to kitchen",
            outcome="completed",
            summary="Navigated to kitchen via corridor B",
        ))
        store.add(Experience(
            objective="inspect loading dock",
            outcome="failed",
            summary="Failed inspection due to blocked access point",
        ))
        store.add(Experience(
            objective="patrol perimeter fence",
            outcome="completed",
            summary="Perimeter patrol completed, fence integrity verified",
        ))
        store.add(Experience(
            objective="charge at dock station",
            outcome="completed",
            summary="Battery fully charged at docking station",
        ))

        # Search for patrol-related experiences
        results = store.search("patrol security check")
        self.assertGreater(len(results), 0)
        # "patrol" experiences should rank higher
        objectives = [r.objective for r in results]
        patrol_found = any("patrol" in obj for obj in objectives[:3])
        self.assertTrue(patrol_found, f"Expected patrol in top results: {objectives}")

    def test_semantic_not_keyword(self) -> None:
        """Verify that similar context terms improve recall, not just exact keywords."""
        store = SemanticExperienceStore()
        store.add(Experience(
            objective="dock for charging",
            outcome="completed",
            summary="Robot docked successfully and battery charging initiated",
        ))
        store.add(Experience(
            objective="patrol the warehouse",
            outcome="completed",
            summary="Warehouse patrol completed without incidents",
        ))
        store.add(Experience(
            objective="low battery emergency return",
            outcome="completed",
            summary="Emergency dock due to critically low battery level",
        ))

        # "battery" should recall both battery-related experiences
        results = store.search("battery low need charging")
        battery_results = [r for r in results if "battery" in r.summary or "charging" in r.summary or "dock" in r.objective]
        self.assertGreaterEqual(len(battery_results), 2)

    def test_at_least_five_experiences_stored_and_recalled(self) -> None:
        """Verify ≥5 experiences can be stored and semantically recalled."""
        store = SemanticExperienceStore()
        experiences = [
            Experience(objective="patrol lobby", outcome="completed", summary="lobby patrol done"),
            Experience(objective="navigate to exit", outcome="completed", summary="reached exit safely"),
            Experience(objective="recognize intruder", outcome="failed", summary="false alarm, no intruder"),
            Experience(objective="dock for charging", outcome="completed", summary="battery charged to full"),
            Experience(objective="follow visitor", outcome="completed", summary="escorted visitor to office"),
            Experience(objective="report smoke alarm", outcome="completed", summary="reported smoke detection"),
        ]
        for exp in experiences:
            store.add(exp)

        self.assertEqual(6, store.size)

        # Semantic search should return relevant results
        results = store.search("patrol security rounds")
        self.assertGreater(len(results), 0)
        self.assertEqual("patrol lobby", results[0].objective)

    def test_search_with_scores(self) -> None:
        store = SemanticExperienceStore()
        store.add(Experience(objective="patrol lobby", outcome="completed", summary="lobby patrol done"))
        store.add(Experience(objective="navigate kitchen", outcome="completed", summary="kitchen navigation done"))

        scored = store.search_with_scores("patrol")
        self.assertGreater(len(scored), 0)
        self.assertGreater(scored[0][1], 0.0)

    def test_long_term_memory_with_semantic_store(self) -> None:
        """LongTermMemory works correctly with SemanticExperienceStore as backend."""
        store = SemanticExperienceStore()
        memory = LongTermMemory(store)

        memory.add(Experience(objective="patrol east wing", outcome="completed", summary="east wing clear"))
        memory.add(Experience(objective="navigate to lab", outcome="completed", summary="arrived at lab"))
        memory.add(Experience(objective="patrol west wing", outcome="failed", summary="blocked by debris"))

        results = memory.search("patrol wing area")
        self.assertGreater(len(results), 0)
        # Both patrol experiences should appear in results
        patrol_results = [r for r in results if "patrol" in r.objective]
        self.assertGreaterEqual(len(patrol_results), 2)


if __name__ == "__main__":
    unittest.main()
