#!/usr/bin/env python3
"""
Drone Tools - narzędzia LangChain powiązane z DroneController API.

Każdy tool odpowiada metodzie z klasy DroneController:
- takeoff() → DroneController.takeoff()
- land() → DroneController.land()
- rtl() → DroneController.rtl()
- goto_relative() → DroneController.send_goto_relative()
- goto_global() → DroneController.send_goto_global()
- set_yaw() → DroneController.send_set_yaw()
- set_speed() → DroneController.set_speed()
- hover() → time.sleep() (brak dedykowanej metody)
"""

from langchain_core.tools import tool


# ============================================================================
# Dozwolone komendy
# ============================================================================

ALLOWED_COMMANDS = {
    "takeoff",
    "land",
    "rtl",
    "goto_relative",
    "goto_global",
    "set_yaw",
    "set_speed",
    "hover"
}


# ============================================================================
# Tool Definitions
# ============================================================================

@tool
def takeoff(altitude: float) -> dict:
    """
    Start the drone and rise to specified altitude.
    Maps to: DroneController.takeoff(altitude)
    
    Args:
        altitude: Target altitude in meters (e.g., 10.0 for 10 meters)
    
    Returns:
        Command dict for mission execution
    """
    return {"command": "takeoff", "altitude": altitude}


@tool
def land() -> dict:
    """
    Land the drone at current position.
    Maps to: DroneController.land()
    
    Returns:
        Command dict for mission execution
    """
    return {"command": "land"}


@tool
def rtl() -> dict:
    """
    Return to launch point (home position).
    Maps to: DroneController.rtl()
    
    Returns:
        Command dict for mission execution
    """
    return {"command": "rtl"}


@tool
def goto_relative(north: float, east: float, down: float = 0.0) -> dict:
    """
    Move drone relative to current position.
    Maps to: DroneController.send_goto_relative(north, east, down)
    
    Args:
        north: Distance in meters (positive = forward/north, negative = backward/south)
        east: Distance in meters (positive = right/east, negative = left/west)
        down: Altitude change in meters (positive = descend, negative = ascend)
    
    Returns:
        Command dict for mission execution
    """
    return {"command": "goto_relative", "north": north, "east": east, "down": down}


@tool
def goto_global(lat: float, lon: float, alt: float) -> dict:
    """
    Move drone to GPS coordinates.
    Maps to: DroneController.send_goto_global(lat, lon, alt)
    
    Args:
        lat: Latitude (e.g., 52.2297)
        lon: Longitude (e.g., 21.0122)
        alt: Altitude in meters
    
    Returns:
        Command dict for mission execution
    """
    return {"command": "goto_global", "lat": lat, "lon": lon, "alt": alt}


@tool
def set_yaw(yaw_degrees: float, relative: bool = True) -> dict:
    """
    Rotate drone to specified heading.
    Maps to: DroneController.send_set_yaw(yaw_rad, relative)
    
    Args:
        yaw_degrees: Rotation angle in degrees (e.g., 90 for 90 degrees right)
        relative: If True, rotate relative to current heading. If False, absolute heading.
    
    Returns:
        Command dict for mission execution
    """
    return {"command": "set_yaw", "yaw": yaw_degrees, "relative": relative}


@tool
def set_speed(speed: float) -> dict:
    """
    Set drone flight speed.
    Maps to: DroneController.set_speed(speed)
    
    Args:
        speed: Speed in meters per second (e.g., 5.0 for 5 m/s)
    
    Returns:
        Command dict for mission execution
    """
    return {"command": "set_speed", "speed": speed}


@tool
def hover(hold_time: float) -> dict:
    """
    Hold current position for specified time.
    Maps to: time.sleep(hold_time) - no dedicated DroneController method
    
    Args:
        hold_time: Time to hover in seconds (e.g., 5.0 for 5 seconds)
    
    Returns:
        Command dict for mission execution
    """
    return {"command": "hover", "hold_time": hold_time}


# ============================================================================
# Lista wszystkich narzędzi do eksportu
# ============================================================================

DRONE_TOOLS = [
    takeoff,
    land,
    rtl,
    goto_relative,
    goto_global,
    set_yaw,
    set_speed,
    hover
]


# ============================================================================
# Helper functions
# ============================================================================

def get_tool_descriptions() -> str:
    """Zwraca opis wszystkich narzędzi jako string do promptu."""
    return "\n".join([
        f"- {t.name}: {t.description.split(chr(10))[0]}" 
        for t in DRONE_TOOLS
    ])


def is_valid_command(command: str) -> bool:
    """Sprawdza czy komenda jest dozwolona."""
    return command in ALLOWED_COMMANDS


def list_tools():
    """Wypisuje wszystkie dostępne narzędzia."""
    print("\n📦 Available DroneController Tools:\n")
    for t in DRONE_TOOLS:
        print(f"  🔧 {t.name}")
        desc_lines = t.description.strip().split('\n')
        print(f"     {desc_lines[0]}")
        # Znajdź linię z "Maps to:"
        for line in desc_lines:
            if "Maps to:" in line:
                print(f"     {line.strip()}")
                break
        print()