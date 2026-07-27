"""Persistent room and object observations used by spatial search."""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
from threading import RLock

from pydantic import BaseModel, Field

from robot_brain.core.world_state import Position


class ObjectObservation(BaseModel):
    room_name: str
    object_name: str
    position: Position
    heading_degrees: float
    confidence: float = Field(ge=0.0, le=1.0)
    bbox: tuple[float, float, float, float] | None = None
    observed_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class RoomMemory(BaseModel):
    name: str
    anchor: Position
    anchor_heading_degrees: float = 0.0
    updated_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class SpatialMemoryStore:
    """Small SQLite repository; safe to point at the main memory database."""

    def __init__(self, database_path: str | Path) -> None:
        self._lock = RLock()
        path = str(database_path)
        if path != ":memory:":
            Path(path).expanduser().parent.mkdir(parents=True, exist_ok=True)
        self._connection = sqlite3.connect(path, check_same_thread=False)
        self._connection.row_factory = sqlite3.Row
        with self._connection:
            self._connection.executescript("""
                CREATE TABLE IF NOT EXISTS spatial_rooms (
                    name TEXT PRIMARY KEY, x REAL NOT NULL, y REAL NOT NULL,
                    heading REAL NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS spatial_observations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT, room_name TEXT NOT NULL,
                    object_name TEXT NOT NULL, x REAL NOT NULL, y REAL NOT NULL,
                    heading REAL NOT NULL, confidence REAL NOT NULL,
                    bbox_json TEXT, observed_at TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS idx_spatial_object
                    ON spatial_observations(object_name, observed_at DESC);
            """)

    def save_room(self, room: RoomMemory) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """INSERT INTO spatial_rooms(name,x,y,heading,updated_at) VALUES(?,?,?,?,?)
                ON CONFLICT(name) DO UPDATE SET x=excluded.x,y=excluded.y,
                heading=excluded.heading,updated_at=excluded.updated_at""",
                (room.name, room.anchor.x, room.anchor.y, room.anchor_heading_degrees,
                 room.updated_at.isoformat()),
            )

    def replace_room_observations(self, room_name: str, items: list[ObjectObservation]) -> None:
        with self._lock, self._connection:
            self._connection.execute("DELETE FROM spatial_observations WHERE room_name=?", (room_name,))
            self._connection.executemany(
                """INSERT INTO spatial_observations
                (room_name,object_name,x,y,heading,confidence,bbox_json,observed_at)
                VALUES(?,?,?,?,?,?,?,?)""",
                [(i.room_name, i.object_name, i.position.x, i.position.y, i.heading_degrees,
                  i.confidence, json.dumps(i.bbox) if i.bbox else None, i.observed_at.isoformat())
                 for i in items],
            )

    def rooms(self) -> list[RoomMemory]:
        with self._lock:
            rows = self._connection.execute("SELECT * FROM spatial_rooms ORDER BY updated_at DESC").fetchall()
        return [RoomMemory(name=r["name"], anchor=Position(x=r["x"], y=r["y"]),
                           anchor_heading_degrees=r["heading"], updated_at=datetime.fromisoformat(r["updated_at"]))
                for r in rows]

    def observations(self, object_name: str, room_name: str | None = None) -> list[ObjectObservation]:
        sql = "SELECT * FROM spatial_observations WHERE lower(object_name)=lower(?)"
        args: list[object] = [object_name]
        if room_name is not None:
            sql += " AND room_name=?"
            args.append(room_name)
        sql += " ORDER BY confidence DESC, observed_at DESC"
        with self._lock:
            rows = self._connection.execute(sql, args).fetchall()
        return [ObjectObservation(room_name=r["room_name"], object_name=r["object_name"],
                                  position=Position(x=r["x"], y=r["y"]), heading_degrees=r["heading"],
                                  confidence=r["confidence"], bbox=json.loads(r["bbox_json"]) if r["bbox_json"] else None,
                                  observed_at=datetime.fromisoformat(r["observed_at"])) for r in rows]

    def close(self) -> None:
        self._connection.close()
