"""Semantic experience store using TF-IDF cosine similarity for recall."""
from __future__ import annotations

import math
import re
from collections import Counter
from datetime import datetime, timezone
from typing import Any, TYPE_CHECKING

from robot_brain.memory.long_term import Experience, ExperienceStore

if TYPE_CHECKING:
    from robot_brain.memory.sqlite_store import SQLiteMemoryStore


def _tokenize(text: str) -> list[str]:
    """Simple whitespace + punctuation tokenizer with lowercasing."""
    return re.findall(r"[a-z0-9一-鿿]+", text.lower())


class TfidfIndex:
    """Lightweight in-process TF-IDF index (no external dependencies)."""

    def __init__(self) -> None:
        self._documents: list[list[str]] = []
        self._df: Counter[str] = Counter()

    @property
    def size(self) -> int:
        return len(self._documents)

    def add(self, tokens: list[str]) -> None:
        self._documents.append(tokens)
        unique_terms = set(tokens)
        for term in unique_terms:
            self._df[term] += 1

    def _tfidf_vector(self, tokens: list[str]) -> dict[str, float]:
        n = len(self._documents) or 1
        tf = Counter(tokens)
        total = len(tokens) or 1
        vector: dict[str, float] = {}
        for term, count in tf.items():
            df = self._df.get(term, 0)
            if df == 0:
                # Term not in corpus — use idf = log(n+1) as a reasonable default
                idf = math.log(n + 1)
            else:
                # Smooth IDF: log((n + 1) / (df + 1)) + 1 to avoid zero when n == df
                idf = math.log((n + 1) / (df + 1)) + 1.0
            vector[term] = (count / total) * idf
        return vector

    def search(self, query_tokens: list[str], limit: int = 5) -> list[tuple[int, float]]:
        """Return (doc_index, cosine_similarity) pairs sorted descending."""
        if not self._documents or not query_tokens:
            return []

        query_vec = self._tfidf_vector(query_tokens)
        results: list[tuple[int, float]] = []

        for idx, doc_tokens in enumerate(self._documents):
            doc_vec = self._tfidf_vector(doc_tokens)
            similarity = self._cosine(query_vec, doc_vec)
            if similarity > 0:
                results.append((idx, similarity))

        results.sort(key=lambda x: x[1], reverse=True)
        return results[:limit]

    @staticmethod
    def _cosine(a: dict[str, float], b: dict[str, float]) -> float:
        common_keys = set(a.keys()) & set(b.keys())
        if not common_keys:
            return 0.0
        dot = sum(a[k] * b[k] for k in common_keys)
        norm_a = math.sqrt(sum(v * v for v in a.values()))
        norm_b = math.sqrt(sum(v * v for v in b.values()))
        if norm_a == 0 or norm_b == 0:
            return 0.0
        return dot / (norm_a * norm_b)


class SemanticExperienceStore:
    """ExperienceStore implementation with TF-IDF semantic recall.

    Satisfies the ExperienceStore Protocol:
      - add(experience: Experience) -> None
      - search(query: str, limit: int = 5) -> list[Experience]
    """

    def __init__(self) -> None:
        self._experiences: list[Experience] = []
        self._index = TfidfIndex()

    @property
    def size(self) -> int:
        return len(self._experiences)

    def add(self, experience: Experience) -> None:
        text = f"{experience.objective} {experience.outcome} {experience.summary}"
        tokens = _tokenize(text)
        self._experiences.append(experience)
        self._index.add(tokens)

    def search(self, query: str, limit: int = 5) -> list[Experience]:
        query_tokens = _tokenize(query)
        if not query_tokens:
            return self._experiences[-limit:]

        results = self._index.search(query_tokens, limit)
        return [self._experiences[idx] for idx, _score in results]

    def search_with_scores(self, query: str, limit: int = 5) -> list[tuple[Experience, float]]:
        """Return experiences with similarity scores for debugging/testing."""
        query_tokens = _tokenize(query)
        if not query_tokens:
            return [(exp, 0.0) for exp in self._experiences[-limit:]]

        results = self._index.search(query_tokens, limit)
        return [(self._experiences[idx], score) for idx, score in results]


class SQLiteSemanticStore:
    """ExperienceStore backed by SQLite for persistence + in-memory TF-IDF index for semantic search.

    On initialization, loads all existing experiences from SQLite into the TF-IDF index.
    New experiences are written to both SQLite and the in-memory index.
    """

    def __init__(self, sqlite_store: "SQLiteMemoryStore") -> None:
        self._sqlite = sqlite_store
        self._index = TfidfIndex()
        self._experiences: list[Experience] = []
        self._load_existing()

    def _load_existing(self) -> None:
        """Load existing experiences from SQLite into the TF-IDF index."""
        # Use the sqlite store's search with a very high limit to get all experiences
        # We use direct SQL to load all experiences
        with self._sqlite._lock:
            rows = self._sqlite._connection.execute(
                "SELECT objective, outcome, summary, created_at FROM experiences ORDER BY created_at, id"
            ).fetchall()
        for row in rows:
            exp = Experience(
                objective=row["objective"],
                outcome=row["outcome"],
                summary=row["summary"],
                created_at=datetime.fromisoformat(row["created_at"]),
            )
            self._experiences.append(exp)
            text = f"{exp.objective} {exp.outcome} {exp.summary}"
            self._index.add(_tokenize(text))

    @property
    def size(self) -> int:
        return len(self._experiences)

    def add(self, experience: Experience) -> None:
        # Persist to SQLite
        self._sqlite.add(experience)
        # Add to in-memory index
        self._experiences.append(experience)
        text = f"{experience.objective} {experience.outcome} {experience.summary}"
        self._index.add(_tokenize(text))

    def search(self, query: str, limit: int = 5) -> list[Experience]:
        query_tokens = _tokenize(query)
        if not query_tokens:
            return self._experiences[-limit:]

        results = self._index.search(query_tokens, limit)
        return [self._experiences[idx] for idx, _score in results]
