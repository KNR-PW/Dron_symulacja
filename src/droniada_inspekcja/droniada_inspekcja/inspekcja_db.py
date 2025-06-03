"""DroneDB – SQLite helper for drone‑inspection missions & reports.

New in this version (2025‑05‑30):
•   get_full_mission(...) – fetch a mission together with all children in one call.
•   get_employees(...), get_infrastructure_changes(...), get_incidents(...),
    get_arucos(...), get_flight_logs(...) – convenience per‑table fetchers.
•   add_flight_log(...) bug‑fix – correctly bind values & JSON‑encode telemetry.
"""
from __future__ import annotations

import json
import sqlite3
from typing import Any, Dict, List, Optional


class DroneDB:
    """SQLite helper for drone‑inspection missions & reports.

    Tables (⇢ = FK):
        missions                1 ⇢ n  employees
                                1 ⇢ n  infrastructure_changes
                                1 ⇢ n  incidents
                                1 ⇢ n  arucos
                                1 ⇢ n  flight_logs

    The schema is lazily created **once per process per DB file** the first time a
    `DroneDB` is constructed for that file.  Subsequent instances skip the DDL so
    you can freely create short‑lived objects in many threads without hammering
    `CREATE TABLE`.
    """

    # ------------------------------------------------------------------
    # class‑level registry so each unique path is initialised only once
    _schemas_initialised: set[str] = set()

    # ---------------------------- life‑cycle ---------------------------
    def __init__(self, db_path: str = "drone_data.db", *, ensure_schema: bool = True) -> None:
        self.db_path = db_path
        if ensure_schema and db_path not in DroneDB._schemas_initialised:
            self._create_tables()               # one‑off DDL
            DroneDB._schemas_initialised.add(db_path)

    # -------------------- low‑level connection helper ------------------
    def _get_connection(self) -> sqlite3.Connection:
        conn = sqlite3.connect(
            self.db_path,
            check_same_thread=False,           # let pools hand a conn to another thread
            timeout=30,
            isolation_level="DEFERRED",      # begin txn only when first write happens
        )
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA foreign_keys = ON")
        conn.execute("PRAGMA journal_mode = WAL")
        conn.execute("PRAGMA synchronous = NORMAL")
        conn.execute("PRAGMA busy_timeout = 30000")
        return conn

    # --------------------------- schema DDL ----------------------------
    def _create_tables(self) -> None:
        """Execute idempotent `CREATE TABLE IF NOT EXISTS` statements."""
        with self._get_connection() as conn:
            c = conn.cursor()

            # master table -------------------------------------------------
            c.executescript(
                """
                CREATE TABLE IF NOT EXISTS missions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    team TEXT NOT NULL,
                    email TEXT NOT NULL,
                    pilot TEXT NOT NULL,
                    phone TEXT NOT NULL,
                    mission_time TEXT NOT NULL,
                    mission_no TEXT NOT NULL,
                    duration TEXT NOT NULL,
                    battery_before TEXT NOT NULL,
                    battery_after TEXT NOT NULL,
                    kp_index INTEGER NOT NULL,
                    infra_map TEXT DEFAULT '/static/img/mapa.jpg'
                );
                CREATE INDEX IF NOT EXISTS idx_missions_time ON missions(mission_time);
                """
            )

            # §3 employees --------------------------------------------------
            c.executescript(
                """
                CREATE TABLE IF NOT EXISTS employees (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    mission_id INTEGER NOT NULL,
                    present TEXT CHECK(present IN ('Jest','Nie ma')),
                    bhp TEXT CHECK(bhp IN ('Tak','Nie')),
                    location TEXT,
                    location_changed TEXT CHECK(location_changed IN ('Tak','Nie')),
                    image TEXT,
                    jury TEXT,
                    FOREIGN KEY (mission_id) REFERENCES missions(id) ON DELETE CASCADE
                );
                CREATE INDEX IF NOT EXISTS idx_employees_mission ON employees(mission_id);
                """
            )

            # §4 infrastructure changes ------------------------------------
            c.executescript(
                """
                CREATE TABLE IF NOT EXISTS infrastructure_changes (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    mission_id INTEGER NOT NULL,
                    category TEXT,
                    detection_time TEXT,
                    location TEXT,
                    image TEXT,
                    jury TEXT,
                    FOREIGN KEY (mission_id) REFERENCES missions(id) ON DELETE CASCADE
                );
                CREATE INDEX IF NOT EXISTS idx_changes_mission ON infrastructure_changes(mission_id);
                """
            )

            # §5 incidents --------------------------------------------------
            c.executescript(
                """
                CREATE TABLE IF NOT EXISTS incidents (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    mission_id INTEGER NOT NULL,
                    event TEXT,
                    event_time TEXT,
                    location TEXT,
                    image TEXT,
                    notified TEXT CHECK(notified IN ('Tak','Nie')),
                    jury TEXT,
                    FOREIGN KEY (mission_id) REFERENCES missions(id) ON DELETE CASCADE
                );
                CREATE INDEX IF NOT EXISTS idx_incidents_mission ON incidents(mission_id);
                """
            )

            # §6 ArUco detections ------------------------------------------
            c.executescript(
                """
                CREATE TABLE IF NOT EXISTS arucos (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    mission_id INTEGER NOT NULL,
                    content TEXT,
                    location TEXT,
                    location_changed TEXT,
                    content_changed TEXT,
                    image TEXT,
                    jury TEXT,
                    FOREIGN KEY (mission_id) REFERENCES missions(id) ON DELETE CASCADE
                );
                CREATE INDEX IF NOT EXISTS idx_arucos_mission ON arucos(mission_id);
                """
            )

            # raw flight telemetry -----------------------------------------
            c.executescript(
                """
                CREATE TABLE IF NOT EXISTS flight_logs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    mission_id INTEGER NOT NULL,
                    timestamp TEXT NOT NULL,
                    telemetry_data TEXT NOT NULL,
                    FOREIGN KEY (mission_id) REFERENCES missions(id) ON DELETE CASCADE
                );
                CREATE INDEX IF NOT EXISTS idx_logs_mission ON flight_logs(mission_id);
                """
            )

    # --------------------- mission CRUD operations ----------------------
    def add_mission(
        self,
        *,
        team: str,
        email: str,
        pilot: str,
        phone: str,
        mission_time: str,
        mission_no: str,
        duration: str,
        battery_before: str,
        battery_after: str,
        kp_index: int,
        infra_map: str = "/static/img/mapa.jpg",
    ) -> int:
        """Insert a mission row and return its auto‑id."""
        with self._get_connection() as conn:
            cur = conn.execute(
                """
                INSERT INTO missions (team, email, pilot, phone, mission_time, mission_no,
                                      duration, battery_before, battery_after, kp_index, infra_map)
                VALUES (?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    team,
                    email,
                    pilot,
                    phone,
                    mission_time,
                    mission_no,
                    duration,
                    battery_before,
                    battery_after,
                    kp_index,
                    infra_map,
                ),
            )
            return cur.lastrowid

    def get_mission(self, mission_id: int) -> Optional[dict[str, Any]]:
        with self._get_connection() as conn:
            row = conn.execute("SELECT * FROM missions WHERE id=?", (mission_id,)).fetchone()
            return dict(row) if row else None

    # ------------------- generic child‑table inserter -------------------
    def _insert_many(self, table: str, rows: List[dict[str, Any]], mission_id: int) -> None:
        if not rows:
            return
        # ensure every dict has the same keys
        keys = list(rows[0].keys())
        required = set(keys)
        mismatched = [i for i, r in enumerate(rows) if set(r.keys()) != required]
        if mismatched:
            raise ValueError(f"Rows {mismatched} in '{table}' have mismatching keys. Expected {required}.")

        columns = ", ".join(keys)
        placeholders = ", ".join(["?"] * len(keys))
        data = [tuple(r[k] for k in keys) for r in rows]
        with self._get_connection() as conn:
            conn.executemany(
                f"INSERT INTO {table} (mission_id, {columns}) VALUES (?, {placeholders})",
                [(mission_id, *d) for d in data],
            )

    # ------------------- child‑table insert wrappers -------------------
    def add_employees(self, mission_id: int, employees: List[dict[str, Any]]) -> None:
        self._insert_many("employees", employees, mission_id)

    def add_infrastructure_changes(self, mission_id: int, changes: List[dict[str, Any]]) -> None:
        self._insert_many("infrastructure_changes", changes, mission_id)

    def add_incidents(self, mission_id: int, incidents: List[dict[str, Any]]) -> None:
        self._insert_many("incidents", incidents, mission_id)

    def add_arucos(self, mission_id: int, arucos: List[dict[str, Any]]) -> None:
        self._insert_many("arucos", arucos, mission_id)

    # ----------------------- flight logs -------------------------------
    def add_flight_log(
        self, mission_id: int, timestamp: str, telemetry_data: Dict[str, List[Any]]
    ) -> int:
        """Insert raw JSON telemetry and return row‑id."""
        with self._get_connection() as conn:
            cur = conn.execute(
                "INSERT INTO flight_logs (mission_id, timestamp, telemetry_data) VALUES (?,?,?)",
                (mission_id, timestamp, json.dumps(telemetry_data)),
            )
            return cur.lastrowid

    # ==================================================================
    #                        NEW READ HELPERS
    # ==================================================================

    # ------------- generic helper -------------
    def _get_children(self, table: str, mission_id: int) -> List[dict[str, Any]]:
        with self._get_connection() as conn:
            cursor = conn.execute(f"SELECT * FROM {table} WHERE mission_id=?", (mission_id,))
            return [dict(r) for r in cursor.fetchall()]

    # ------------- per‑table wrappers ----------
    def get_employees(self, mission_id: int) -> List[dict[str, Any]]:
        return self._get_children("employees", mission_id)

    def get_infrastructure_changes(self, mission_id: int) -> List[dict[str, Any]]:
        return self._get_children("infrastructure_changes", mission_id)

    def get_incidents(self, mission_id: int) -> List[dict[str, Any]]:
        return self._get_children("incidents", mission_id)

    def get_arucos(self, mission_id: int) -> List[dict[str, Any]]:
        return self._get_children("arucos", mission_id)

    def get_flight_logs(self, mission_id: int) -> List[dict[str, Any]]:
        return self._get_children("flight_logs", mission_id)

    # ------------- mission + all children ------
    def get_full_mission(self, mission_id: int) -> Optional[dict[str, Any]]:
        """Return mission record with all its child rows embedded as lists."""
        mission = self.get_mission(mission_id)
        if mission is None:
            return None

        # pull children in a single connection
        with self._get_connection() as conn:
            for table in (
                "employees",
                "infrastructure_changes",
                "incidents",
                "arucos",
                "flight_logs",
            ):
                mission[table] = [dict(r) for r in conn.execute(
                    f"SELECT * FROM {table} WHERE mission_id=?", (mission_id,)
                ).fetchall()]
        return mission

# db = DroneDB()
# # Example usage: add employees to a mission
# mission_id = db.add_mission(
#     team="Team Alpha",
#     email="alpha@example.com",
#     pilot="John Doe",
#     phone="123456789",
#     mission_time="2024-06-01 10:00",
#     mission_no="M001",
#     duration="30m",
#     battery_before="100%",
#     battery_after="80%",
#     kp_index=1,
# )

# employees = [
#     {
#         "present": "Jest",
#         "bhp": "Tak",
#         "location": "Hangar",
#         "location_changed": "Nie",
#         "image": "/static/img/employee1.jpg",
#         "jury": "None"
#     },
#     {
#         "present": "Nie ma",
#         "bhp": "Nie",
#         "location": "Field",
#         "location_changed": "Tak",
#         "image": "/static/img/employee2.jpg",
#         "jury": "None"
#     }
# ]

# db.add_employees(mission_id, employees)
# m = db.get_mission(mission_id)
# print(m)