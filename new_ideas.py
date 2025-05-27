import random
from datetime import datetime

def transform_detections(detections):
    """
    Transform detections from database format to the desired list format.
    
    Args:
        detections: List of dictionaries representing detections from the database.
        
    Returns:
        Dictionary with categorized and formatted detections.
    """
    now = datetime.now()
    
    # Initialize lists for each category
    infrastructure_changes = []
    incidents = []
    arucos = []
    
    for detection in detections:
        # Extract detection data
        category = detection['category']
        timestamp = detection['timestamp']
        latitude = detection['latitude']
        longitude = detection['longitude']
        picture = detection['picture']
        bhp = detection['bhp']
        worker = detection['worker']
        change = detection['change']
        
        # Determine category and format accordingly
        if category == 'Infrastructure Change':
            infrastructure_changes.append({
                "category": random.choice([
                    "Rurociąg", "Linia wysokiego napięcia", "Ogrodzenie", "Pozostawiony sprzęt"
                ]),
                "detection_time": timestamp.strftime("%d/%m/%Y, %H:%M:%S"),
                "location": f"Lat {latitude}, Long {longitude}",
                "image": picture if picture else "/static/img/pipe.jpg",
                "jury": "+" if bhp else "-"
            })
        elif category == 'Incident':
            incidents.append({
                "event": "Intruz",
                "time": timestamp.strftime("%d/%m/%Y, %H:%M:%S"),
                "location": f"Lat {latitude}, Long {longitude}",
                "image": picture if picture else "/static/img/intruder.jpg",
                "notified": "Tak" if worker else "Nie",
                "jury": "+" if change else "-"
            })
        elif category == 'Aruco':
            arucos.append({
                "content": f"{random.randint(10, 99)}",
                "location": f"Lat {latitude}, Long {longitude}",
                "location_changed": "Tak" if change else "Nie",
                "content_changed": random.choice(["Tak", "Nie", "LiczbaData Tak/Nie"]),
                "image": picture if picture else f"/static/img/aruco{random.randint(1, 2)}.png",
                "jury": "+" if bhp else "-"
            })
    
    # Combine categorized detections into a single dictionary
    return {
        "infrastructure_changes": infrastructure_changes,
        "incidents": incidents,
        "arucos": arucos
    }

# Example usage
if __name__ == "__main__":
    # Fetch detections from the database
    db = DroneDB('/path/to/drone_data.db')
    detections = db.get_all_detections()
    
    # Transform detections
    transformed_data = transform_detections(detections)
    
    # Print the transformed data
    print(json.dumps(transformed_data, indent=4))
