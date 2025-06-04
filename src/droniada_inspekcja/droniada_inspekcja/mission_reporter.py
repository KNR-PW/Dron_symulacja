import rclpy
from rclpy.node import Node
from drone_interfaces.srv import CreateReport, UpdateReport
from datetime import datetime
import json
import sqlite3

from droniada_inspekcja.inspekcja_db import DroneDB


class MissionReporter(Node):
    def __init__(self):
        super().__init__('mission_reporter')

        # Parameter: path to the same SQLite database used by your DroneDB wrapper
        self.declare_parameter('db_path', 'drone_data.db')
        db_path = self.get_parameter('db_path').get_parameter_value().string_value

        # Initialize the DroneDB instance
        self.db = DroneDB(db_path)

        # ROS2 service clients for create_report and update_report
        self.create_client_ = self.create_client(CreateReport, 'create_report')
        self.update_client_ = self.create_client(UpdateReport, 'update_report')

        # Wait for both services to become available
        self.get_logger().info('Waiting for create_report and update_report services...')
        self.create_client_.wait_for_service()
        self.update_client_.wait_for_service()
        self.get_logger().info('Services are now available.')

        # Keep track of which mission_id we last pushed to the website
        self.last_published_mission_id = None

        # Timer: call every 60 seconds (adjust as needed)
        timer_period_sec = 15.0
        self.create_timer(timer_period_sec, self.timer_callback)

        self.get_logger().info(f'MissionReporter will poll every {timer_period_sec} seconds.')

    def timer_callback(self):
        """
        Called periodically. Reads the highest mission ID from the database.
        If it's a new mission, calls create_report. Otherwise, calls update_report
        with the full mission payload.
        """
        try:
            # 1) Find the maximum mission ID in the missions table
            with self.db._get_connection() as conn:
                row = conn.execute("SELECT MAX(id) AS maxid FROM missions").fetchone()
                latest_id = row["maxid"] if row and row["maxid"] is not None else None

            if latest_id is None:
                # No missions in the DB yet
                self.get_logger().debug("No missions found in database.")
                return

            self.get_logger().info(f"Latest mission ID in DB: {latest_id}")
            self.get_logger().info(f"Latest published mission ID: {self.last_published_mission_id}")

            # 2) If we have never published any mission before, or we have a new mission:
            if self.last_published_mission_id is None or latest_id != self.last_published_mission_id:
                # Fetch the full mission record (including children)
                mission = self.db.get_full_mission(latest_id)
                if mission is None:
                    self.get_logger().warn(f"Mission {latest_id} was not found by get_full_mission().")
                    return

                # Build the payload for the website API
                payload = self.build_website_payload(mission)

                # If no mission has ever been published, use create_report.
                # Otherwise, a new mission appeared; also use create_report for the new ID.
                self.call_create_report(payload)
                self.last_published_mission_id = latest_id

            else:
                # Same mission as last time: fetch it and re-publish via update_report
                mission = self.db.get_full_mission(latest_id)
                if mission is None:
                    self.get_logger().warn(f"Mission {latest_id} was not found by get_full_mission().")
                    return

                payload = self.build_website_payload(mission)
                image_paths = self.get_all_image_paths(mission)
                self.get_logger().info(f"Updating report for missionwith {image_paths} images.")
                self.call_update_report(payload, images_list=image_paths)

        except Exception as e:
            self.get_logger().error(f"Exception in timer_callback: {e}", exc_info=True)

    def build_website_payload(self, mission: dict) -> dict:
        """
        Convert the mission dict (as returned by get_full_mission) into the JSON
        payload that the website's /api/report/create (and /update) expect.

        The website build_report(...) expects:
          - team, email, pilot, phone, mission_time (string "dd/mm/YYYY, HH:MM:SS"),
            mission_no, duration, battery_before, battery_after, kp_index,
            employees (list), infrastructure_changes (list), incidents (list),
            arucos (list), infra_map.

        The 'mission' dict includes extra keys (e.g. 'id', 'flight_logs').
        We strip out 'id' and 'flight_logs' and reformat mission_time.
        """
        # Extract and reformat mission_time:
        raw_time = mission.get("mission_time", "")
        formatted_time = ""
        if raw_time:
            # Try parsing "YYYY-MM-DD HH:MM:SS" or "YYYY-MM-DD HH:MM"
            for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M"):
                try:
                    dt = datetime.strptime(raw_time, fmt)
                    formatted_time = dt.strftime("%d/%m/%Y, %H:%M:%S")
                    break
                except ValueError:
                    continue
            else:
                # If parsing fails, just use current time in the required format
                now = datetime.now()
                formatted_time = now.strftime("%d/%m/%Y, %H:%M:%S")
                self.get_logger().warn(f"Could not parse mission_time '{raw_time}', using current time.")

        payload = {
            "team": mission.get("team", ""),
            "email": mission.get("email", ""),
            "pilot": mission.get("pilot", ""),
            "phone": mission.get("phone", ""),
            "mission_time": formatted_time,
            "mission_no": mission.get("mission_no", ""),
            "duration": mission.get("duration", ""),
            "battery_before": mission.get("battery_before", ""),
            "battery_after": mission.get("battery_after", ""),
            "kp_index": mission.get("kp_index", 0),
            "employees": mission.get("employees", []),
            "infrastructure_changes": mission.get("infrastructure_changes", []),
            "incidents": mission.get("incidents", []),
            "arucos": mission.get("arucos", []),
            # If the mission dict has 'infra_map', use it; otherwise fallback
            "infra_map": mission.get("infra_map", "/static/img/mapa.jpg"),
        }

        return payload

    def call_create_report(self, payload: dict):
        """
        Call the /create_report service with the given payload (as dict).
        """
        req = CreateReport.Request()
        req.json_report = json.dumps(payload)

        future = self.create_client_.call_async(req)

        def _on_create_done(fut):
            try:
                res = fut.result()
                if res.success:
                    self.get_logger().info(f"create_report succeeded: {res.message}")
                else:
                    self.get_logger().warn(f"create_report failed: {res.message}")
            except Exception as e:
                self.get_logger().error(f"Exception calling create_report service: {e}", exc_info=True)

        future.add_done_callback(_on_create_done)

    def get_all_image_paths(self, mission: dict):
        image_paths = []
        # Aruco images
        for aruco in mission.get("arucos", []):
            img_path = aruco.get("image_path")
            if img_path:
                image_paths.append(img_path)
        # People images
        for person in mission.get("employees", []):
            img_path = person.get("image_path")
            if img_path:
                image_paths.append(img_path)
        # Incident/event images
        for incident in mission.get("incidents", []):
            img_path = incident.get("image_path")
            if img_path:
                image_paths.append(img_path)
        return image_paths

    def call_update_report(self, payload: dict, images_list=None):
        """
        Call the /update_report service with the given payload (as dict) and images_list.
        """
        req = UpdateReport.Request()
        req.json_update = json.dumps(payload)
        if images_list is not None:
            req.images_list = images_list
        future = self.update_client_.call_async(req)

        def _on_update_done(fut):
            try:
                res = fut.result()
                if res.success:
                    self.get_logger().info(f"update_report succeeded: {res.message}")
                else:
                    self.get_logger().warn(f"update_report failed: {res.message}")
            except Exception as e:
                self.get_logger().error(f"Exception calling update_report service: {e}", exc_info=True)

        future.add_done_callback(_on_update_done)


def main(args=None):
    rclpy.init(args=args)
    node = MissionReporter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
