#!/usr/bin/env python3
"""
import_persons.py

Reads a JSON file containing per-frame detection outputs (with "person" objects)
and inserts each detected person into the `employees` table of a DroneDB SQLite database.

Usage:
    python3 import_persons.py \
        --json_file path/to/all_detections.json \
        --db_path path/to/drone_data.db \
        --mission_id 1 \
        [--photo_dir path/to/photo_dir]

Arguments:
    --json_file   Path to a JSON file that is either:
                     • An array of frame‐output objects (each with keys "frame" and "objects"), or
                     • A single frame‐output object (with "frame" and "objects").
    --db_path     Path to the SQLite database file (DroneDB). If it does not yet exist,
                  DroneDB will create it and initialize the schema.
    --mission_id  The mission ID under which to insert all employee (person) rows.
    --photo_dir   (Optional) Directory where per‐frame JPEGs are saved. Default: "/home/knr/Dron_symulacja/hailo/photo"
"""

import argparse
import json
import os
import sys

from drone_db import DroneDB

def parse_args():
    parser = argparse.ArgumentParser(
        description="Import detected persons from JSON into DroneDB.employees"
    )
    parser.add_argument(
        "--json_file",
        required=True,
        help="Path to the JSON file containing detection outputs",
    )
    parser.add_argument(
        "--db_path",
        default="drone_data.db",
        help="Path to the SQLite DB file (DroneDB). Default: drone_data.db",
    )
    parser.add_argument(
        "--mission_id",
        required=True,
        type=int,
        help="Existing mission ID under which to insert employee rows",
    )
    parser.add_argument(
        "--photo_dir",
        default="/home/knr/Dron_symulacja/hailo/photo",
        help="Directory where per-frame JPEGs are stored. Default: /home/knr/Dron_symulacja/hailo/photo",
    )
    return parser.parse_args()


def load_json(json_path):
    """Load JSON from the given path. The JSON can be either:
       - A list of frame objects (each with "frame" and "objects"), or
       - A single frame object (dict with "frame" and "objects").
       
       Returns a list of frame dicts."""
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, list):
        return data
    elif isinstance(data, dict):
        return [data]
    else:
        raise ValueError(f"Expected JSON list or dict, got {type(data)}")


def extract_person_rows(frames, photo_dir):
    """
    From a list of frame dictionaries, extract one dict per detected person,
    matching the DroneDB.employees schema (except mission_id).

    Each frame dict is expected to have:
      {
        "frame": <int>,
        "objects": [
            {
              "class": "person",
              "id": <int>,
              "x": <float>,
              "y": <float>,
              "width": <float>,
              "height": <float>,
              "hat": "hat"/"no_hat"/"none",
              "vest": "vest"/"no_vest"/"none"
            },
            { ... other objects (hats, vests) ... }
        ]
      }
    """
    employees = []

    for frame in frames:
        frame_num = frame.get("frame")
        if frame_num is None:
            print(f"[Warning] Frame object missing 'frame' key: {frame}", file=sys.stderr)
            continue

        objects = frame.get("objects", [])
        for obj in objects:
            if obj.get("class") != "person":
                continue

            pid = obj.get("id", -1)
            x = obj.get("x", 0.0)
            y = obj.get("y", 0.0)
            w = obj.get("width", 0.0)
            h = obj.get("height", 0.0)
            hat_label = obj.get("hat", "none")
            vest_label = obj.get("vest", "none")

            # present = "Jest" (always detected)
            present = "Jest"

            # bhp = "Tak" if they have a hat or vest, else "Nie"
            has_hat = (hat_label == "hat")
            has_vest = (vest_label == "vest")
            bhp = "Tak" if (has_hat or has_vest) else "Nie"

            # location as a string
            location = f"frame:{frame_num} bbox:[{int(x)},{int(y)},{int(w)},{int(h)}]"

            # For this simple import, we never track movement, so
            location_changed = "Nie"

            # Determine image path if the frame JPEG exists
            img_filename = f"{frame_num}.jpg"
            img_path = os.path.join(photo_dir, img_filename)
            if not os.path.isfile(img_path):
                # If the JPEG is missing, store an empty string
                print(f"[Info] JPEG not found for frame {frame_num}: expected at {img_path}", file=sys.stderr)
                img_path_to_store = ""
            else:
                img_path_to_store = img_path

            # jury default: "None"
            jury = "None"

            row = {
                "present": present,
                "bhp": bhp,
                "location": location,
                "location_changed": location_changed,
                "image": img_path_to_store,
                "jury": jury
            }
            employees.append(row)

    return employees


def main():
    args = parse_args()

    # 1) Load JSON frames
    try:
        frames = load_json(args.json_file)
    except Exception as e:
        print(f"[Error] Failed to load JSON file {args.json_file}: {e}", file=sys.stderr)
        sys.exit(1)

    if not frames:
        print("[Warning] No frames found in JSON. Exiting without inserting any employees.", file=sys.stderr)
        sys.exit(0)

    # 2) Extract one row per "person"
    employees = extract_person_rows(frames, args.photo_dir)
    if not employees:
        print("[Info] No 'person' objects found in JSON. Nothing to insert.", file=sys.stderr)
        sys.exit(0)

    # 3) Initialize DroneDB and insert
    db = DroneDB(args.db_path, ensure_schema=True)

    # Verify that the mission exists
    mission = db.get_mission(args.mission_id)
    if mission is None:
        print(f"[Error] No mission found with id={args.mission_id} in {args.db_path}", file=sys.stderr)
        sys.exit(1)

    # Insert all employee rows under this mission
    try:
        db.add_employees(args.mission_id, employees)
    except Exception as e:
        print(f"[Error] Failed to insert employees into DB: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Successfully inserted {len(employees)} employee records under mission_id={args.mission_id}.")


if __name__ == "__main__":
    main()
