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
    map_id: str = "legacy"
    map_version: str | None = None
    frame_id: str = "world"
    session_id: str | None = None
    persistent_map: bool = False
    pose_kind: str = "observation_pose"


class RoomMemory(BaseModel):
    name: str
    anchor: Position
    anchor_heading_degrees: float = 0.0
    updated_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))
    map_id: str = "legacy"
    map_version: str | None = None
    frame_id: str = "world"
    session_id: str | None = None
    persistent_map: bool = False


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
                    name TEXT NOT NULL, x REAL NOT NULL, y REAL NOT NULL,
                    heading REAL NOT NULL, updated_at TEXT NOT NULL,
                    map_id TEXT NOT NULL DEFAULT 'legacy', map_version TEXT,
                    frame_id TEXT NOT NULL DEFAULT 'world', session_id TEXT,
                    persistent_map INTEGER NOT NULL DEFAULT 0,
                    PRIMARY KEY(map_id, name)
                );
                CREATE TABLE IF NOT EXISTS spatial_observations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT, room_name TEXT NOT NULL,
                    object_name TEXT NOT NULL, x REAL NOT NULL, y REAL NOT NULL,
                    heading REAL NOT NULL, confidence REAL NOT NULL,
                    bbox_json TEXT, observed_at TEXT NOT NULL,
                    map_id TEXT NOT NULL DEFAULT 'legacy', map_version TEXT,
                    frame_id TEXT NOT NULL DEFAULT 'world', session_id TEXT,
                    persistent_map INTEGER NOT NULL DEFAULT 0,
                    pose_kind TEXT NOT NULL DEFAULT 'observation_pose'
                );
                CREATE INDEX IF NOT EXISTS idx_spatial_object
                    ON spatial_observations(object_name, observed_at DESC);
            """)
        self._migrate_schema()

    def _migrate_schema(self) -> None:
        room_columns = {
            "map_id": "TEXT NOT NULL DEFAULT 'legacy'",
            "map_version": "TEXT",
            "frame_id": "TEXT NOT NULL DEFAULT 'world'",
            "session_id": "TEXT",
            "persistent_map": "INTEGER NOT NULL DEFAULT 0",
        }
        observation_columns = {
            **room_columns,
            "pose_kind": "TEXT NOT NULL DEFAULT 'observation_pose'",
        }
        with self._connection:
            for table, columns in (
                ("spatial_rooms", room_columns),
                ("spatial_observations", observation_columns),
            ):
                existing = {
                    row["name"]
                    for row in self._connection.execute(f"PRAGMA table_info({table})")
                }
                for name, declaration in columns.items():
                    if name not in existing:
                        self._connection.execute(
                            f"ALTER TABLE {table} ADD COLUMN {name} {declaration}"
                        )
            room_info = list(self._connection.execute("PRAGMA table_info(spatial_rooms)"))
            primary_key = [row["name"] for row in room_info if row["pk"]]
            if primary_key == ["name"]:
                self._connection.executescript("""
                    ALTER TABLE spatial_rooms RENAME TO spatial_rooms_legacy_pk;
                    CREATE TABLE spatial_rooms (
                        name TEXT NOT NULL, x REAL NOT NULL, y REAL NOT NULL,
                        heading REAL NOT NULL, updated_at TEXT NOT NULL,
                        map_id TEXT NOT NULL DEFAULT 'legacy', map_version TEXT,
                        frame_id TEXT NOT NULL DEFAULT 'world', session_id TEXT,
                        persistent_map INTEGER NOT NULL DEFAULT 0,
                        PRIMARY KEY(map_id, name)
                    );
                    INSERT INTO spatial_rooms
                    SELECT name,x,y,heading,updated_at,map_id,map_version,frame_id,
                           session_id,persistent_map FROM spatial_rooms_legacy_pk;
                    DROP TABLE spatial_rooms_legacy_pk;
                """)

    def save_room(self, room: RoomMemory) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """INSERT INTO spatial_rooms
                (name,x,y,heading,updated_at,map_id,map_version,frame_id,session_id,persistent_map)
                VALUES(?,?,?,?,?,?,?,?,?,?)
                ON CONFLICT(map_id,name) DO UPDATE SET x=excluded.x,y=excluded.y,
                heading=excluded.heading,updated_at=excluded.updated_at,
                map_id=excluded.map_id,map_version=excluded.map_version,
                frame_id=excluded.frame_id,session_id=excluded.session_id,
                persistent_map=excluded.persistent_map""",
                (room.name, room.anchor.x, room.anchor.y, room.anchor_heading_degrees,
                 room.updated_at.isoformat(), room.map_id, room.map_version, room.frame_id,
                 room.session_id, int(room.persistent_map)),
            )

    def replace_room_observations(
        self,
        room_name: str,
        items: list[ObjectObservation],
        *,
        map_id: str | None = None,
    ) -> None:
        map_id = map_id or (items[0].map_id if items else "legacy")
        if any(item.map_id != map_id for item in items):
            raise ValueError("room observations must belong to one map")
        with self._lock, self._connection:
            self._connection.execute(
                "DELETE FROM spatial_observations WHERE room_name=? AND map_id=?",
                (room_name, map_id),
            )
            self._connection.executemany(
                """INSERT INTO spatial_observations
                (room_name,object_name,x,y,heading,confidence,bbox_json,observed_at,
                 map_id,map_version,frame_id,session_id,persistent_map,pose_kind)
                VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                [(i.room_name, i.object_name, i.position.x, i.position.y, i.heading_degrees,
                  i.confidence, json.dumps(i.bbox) if i.bbox else None, i.observed_at.isoformat(),
                  i.map_id, i.map_version, i.frame_id, i.session_id, int(i.persistent_map),
                  i.pose_kind)
                 for i in items],
            )

    def rooms(self, *, map_id: str | None = None) -> list[RoomMemory]:
        sql = "SELECT * FROM spatial_rooms"
        args: list[object] = []
        if map_id is not None:
            sql += " WHERE map_id=?"
            args.append(map_id)
        sql += " ORDER BY updated_at DESC"
        with self._lock:
            rows = self._connection.execute(sql, args).fetchall()
        return [RoomMemory(name=r["name"], anchor=Position(x=r["x"], y=r["y"]),
                           anchor_heading_degrees=r["heading"], updated_at=datetime.fromisoformat(r["updated_at"]),
                           map_id=r["map_id"], map_version=r["map_version"], frame_id=r["frame_id"],
                           session_id=r["session_id"], persistent_map=bool(r["persistent_map"]))
                for r in rows]

    def observations(self, object_name: str, room_name: str | None = None,
                     *, map_id: str | None = None) -> list[ObjectObservation]:
        sql = "SELECT * FROM spatial_observations WHERE lower(object_name)=lower(?)"
        args: list[object] = [object_name]
        if room_name is not None:
            sql += " AND room_name=?"
            args.append(room_name)
        if map_id is not None:
            sql += " AND map_id=?"
            args.append(map_id)
        sql += " ORDER BY confidence DESC, observed_at DESC"
        with self._lock:
            rows = self._connection.execute(sql, args).fetchall()
        return [ObjectObservation(room_name=r["room_name"], object_name=r["object_name"],
                                  position=Position(x=r["x"], y=r["y"]), heading_degrees=r["heading"],
                                  confidence=r["confidence"], bbox=json.loads(r["bbox_json"]) if r["bbox_json"] else None,
                                  observed_at=datetime.fromisoformat(r["observed_at"]), map_id=r["map_id"],
                                  map_version=r["map_version"], frame_id=r["frame_id"], session_id=r["session_id"],
                                  persistent_map=bool(r["persistent_map"]), pose_kind=r["pose_kind"]) for r in rows]

    def close(self) -> None:
        self._connection.close()
