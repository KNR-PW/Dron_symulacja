"""
Launch: suas_simple_mission — arm -> takeoff -> podlot nad namiot -> RTL.

Wymaga dzialajacego drone_handler (gazeboo_ap_sim.launch.py / hardware)
oraz detektora publikujacego /tent_detections (suas_detect_gazebo albo
suas_detect_jetson). Startuje sam z siebie — bez gui_panel.

Domyslne wartosci sa dla REALU. Kamera w Gazebo (gimbal_small_3d) ma juz ten
sam FOV (64.4 deg) i klatke 1024x1024, wiec w symulacji nadpisuje sie TYLKO
katy gimbala (inna kalibracja: -45 = prosto w dol, +45 = przod):
  ros2 launch drone_bringup suas_simple_mission.launch.py \
      pitch_search:=30.0 pitch_min:=-45.0 pitch_max:=45.0 pitch_hover_thr:=-38.0 \
      det_confirm_gap:=1.5
Detektor w Gazebo chodzi ~2-4 FPS (na realu 7-14), wiec domyslne
det_confirm_gap:=0.5 kasuje okno potwierdzania po kazdej klatce — stad 1.5.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# (nazwa, domyslna wartosc) — wartosci kontrolera MUSZA byc takie same jak w
# suas_flight_controller.launch.py, bo to ten sam kod sterowania.
PARAMS = [
    # ─── Misja (tylko suas_simple_mission) ────────────────
    ("settle_time",     "3.0"),    # zawis po starcie przed szukaniem [s]
    ("hover_hold_time", "5.0"),    # ile zawis nad namiotem musi trwac [s]
    ("center_tol",      "0.12"),   # jak blisko srodka kadru ma byc namiot (0..1)
    ("search_timeout",  "120.0"),  # limit na caly podlot, potem powrot [s]
    ("finish_action",   "rtl"),    # rtl | land | none

    # ─── Kontroler (jak w suas_flight_controller.launch.py) ─
    ("target_alt",   "30.0"),       # wysokosc startu I trzymana w locie
    # Zmierzone dla OAK-D PRO W. Crop do kwadratu tnie boki, nie gore/dol,
    # wiec pionowy FOV jest ten sam co przy klatce 4:3 -> vfov bez zmian.
    ("vfov_deg",     "64.4"),
    # Kwadratowe preview 1024x1024 z config/oak_rgb.yaml
    ("img_w",        "1024"),
    ("img_h",        "1024"),
    ("damping",      "0.6"),
    ("kp_vx",        "4.0"),
    ("kp_vy",        "1.0"),
    ("kp_alt",       "0.5"),
    ("kp_yaw",       "0.3"),
    ("max_vel",      "3.0"),
    ("max_vz",       "1.5"),
    ("max_yaw_rate", "0.5"),
    ("ema_alpha",    "0.15"),
    ("lost_timeout",    "3.0"),
    ("hover_deadzone",  "0.08"),
    ("gimbal_deadzone", "0.06"),
    # Filtr falszywych detekcji: M trafien z ostatnich N klatek przed APPROACH
    ("det_confirm_frames", "4"),
    ("det_window_frames",  "8"),
    ("det_confirm_gap",    "0.5"),
    # Prog pewnosci: parametr 'conf' w launchach detekcji, nie tutaj
    ("require_same_track", "true"),
    # Konwencja realu: 0 = poziomo, -45 = pod katem w dol/przod, -90 = prosto w dol
    ("pitch_search",    "-45.0"),
    ("pitch_min",       "-90.0"),
    ("pitch_max",       "-30.0"),
    ("pitch_hover_thr", "-83.0"),
]


def generate_launch_description():
    args = [DeclareLaunchArgument(name, default_value=default)
            for name, default in PARAMS]

    mission = Node(
        package="drone_autonomy",
        executable="suas_simple_mission",
        output="screen",
        parameters=[{name: LaunchConfiguration(name) for name, _ in PARAMS}],
    )

    return LaunchDescription(args + [mission])
