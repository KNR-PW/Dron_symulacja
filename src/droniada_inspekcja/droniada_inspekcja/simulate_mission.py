import time
import random
import json
from datetime import datetime, timedelta

from droniada_inspekcja.inspekcja_db import DroneDB  # Adjust import path if needed


def generate_dummy_telemetry(step: int) -> dict[str, list]:
    """
    Generate a simple telemetry_data dictionary for a given time step.
    Returns a dict where each key maps to a list of a single sample value.
    """
    # Example telemetry: altitude rises, battery drops, position drifts
    altitude = 100 + step * 5  # in meters
    battery = max(0, 100 - step * 5)  # percentage
    lat = 50.0 + step * 0.0005
    lon = 19.0 - step * 0.0005

    return {
        "altitude": [altitude],
        "battery": [battery],
        "lat": [lat],
        "lon": [lon],
    }


def main():
    # 1) Initialize the database (uses "drone_data.db" by default)
    db = DroneDB("drone_data.db")

    # 2) Create a new mission
    mission_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    mission_id = db.add_mission(
        team="SimTeam",
        email="sim@example.com",
        pilot="Simulator",
        phone="000-0000",
        mission_time=mission_time_str,
        mission_no="SIM-001",
        duration="0m",  # will update later if desired
        battery_before="100%",
        battery_after="100%",
        kp_index=42,
        infra_map="/static/img/mapa.jpg",
    )
    print(f"[{datetime.now().isoformat()}] Created mission ID {mission_id}")

    # 3) Add two dummy employees
    employees = [
        {
            "present": "Jest",
            "bhp": "Tak",
            "location": "Base",
            "location_changed": "Nie",
            "image": "/static/img/emp1.jpg",
            "jury": "None",
        },
        {
            "present": "Nie ma",
            "bhp": "Nie",
            "location": "Field",
            "location_changed": "Tak",
            "image": "/static/img/emp2.jpg",
            "jury": "None",
        },
    ]
    db.add_employees(mission_id, employees)
    print(f"[{datetime.now().isoformat()}] Added employees for mission {mission_id}")

    # 4) Simulation parameters
    total_steps = 10
    step_interval_sec = 2  # seconds between each simulated log entry

    # 5) Loop to simulate mission progress
    for step in range(1, total_steps + 1):
        # 5.1) Compute timestamp for this log entry
        log_time = datetime.now() + timedelta(seconds=step * step_interval_sec)
        timestamp_str = log_time.strftime("%Y-%m-%d %H:%M:%S")

        # 5.2) Generate telemetry and insert into flight_logs
        telemetry = generate_dummy_telemetry(step)
        log_id = db.add_flight_log(mission_id, timestamp=timestamp_str, telemetry_data=telemetry)
        print(f"[{datetime.now().isoformat()}] Added flight log {log_id} at {timestamp_str}: {telemetry}")

        # 5.3) At step 3, insert an infrastructure change
        if step == 3:
            infra_change = [
                {
                    "category": "Obstacle",
                    "detection_time": timestamp_str,
                    "location": "Location A",
                    "image": "/static/img/obstacle.jpg",
                    "jury": "None",
                }
            ]
            db.add_infrastructure_changes(mission_id, infra_change)
            print(f"[{datetime.now().isoformat()}] Added infrastructure change at step {step}")

        # 5.4) At step 5, insert an incident
        if step == 5:
            incident = [
                {
                    "event": "Minor collision",
                    "event_time": timestamp_str,
                    "location": "Sector B",
                    "image": "/static/img/incident.jpg",
                    "notified": "Tak",
                    "jury": "None",
                }
            ]
            db.add_incidents(mission_id, incident)
            print(f"[{datetime.now().isoformat()}] Added incident at step {step}")

        # 5.5) At step 7, insert an ArUco detection
        if step == 7:
            aruco = [
                {
                    "content": "ArUco-42",
                    "location": "Point C",
                    "location_changed": "Nie",
                    "content_changed": "Nie",
                    "image": "/static/img/aruco.jpg",
                    "jury": "None",
                }
            ]
            db.add_arucos(mission_id, aruco)
            print(f"[{datetime.now().isoformat()}] Added ArUco detection at step {step}")

        # Sleep to simulate time passing
        time.sleep(step_interval_sec)

    # 6) After simulation, update mission duration and battery_after
    #    (Wrapper has no update_mission, so we'll delete and recreate,
    #     or assume mission is complete for demonstration)
    print(f"[{datetime.now().isoformat()}] Simulation complete for mission {mission_id}")
    print("Final mission state:")
    mission = db.get_full_mission(mission_id)
    print(json.dumps(mission, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    print("Starting mission simulation...")
    main()
