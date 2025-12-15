import json
import socketio
import eventlet
import paho.mqtt.client as mqtt

# ----------------------------------------------------
# Configuration
# ----------------------------------------------------

MQTT_HOST = "f4209e15de424182b7f1f41170484e60.s1.eu.hivemq.cloud"
MQTT_PORT = 8883

USERNAME = "Knr_web"
PASSWORD = "Drony123"

# List of MQTT topics you want to forward
MQTT_TOPICS = [
    "drone/pose",
    "drone/velocity",
    "battery/state",
    # add more topics here
]

# Socket.IO server
sio = socketio.Server(cors_allowed_origins="*")
app = socketio.WSGIApp(sio)

SOCKETIO_PORT = 5000

# ----------------------------------------------------
# MQTT CALLBACKS
# ----------------------------------------------------

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    for topic in MQTT_TOPICS:
        print(f"Subscribing to {topic}")
        client.subscribe(topic)

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        try:
            data = json.loads(payload)
        except:
            data = payload  # if not JSON, send raw string

        # Emit message to all socket.io clients
        sio.emit(
            "telemetry",
            {
                "topic": msg.topic,
                "data": data
            }
        )
        print(f"[MQTT] {msg.topic}: {payload}")

    except Exception as e:
        print(f"Error processing message: {e}")

# ----------------------------------------------------
# MQTT CLIENT SETUP
# ----------------------------------------------------

mqtt_client = mqtt.Client()

# Secure comms + login
mqtt_client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
mqtt_client.username_pw_set(USERNAME, PASSWORD)

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

mqtt_client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)

# MQTT loop runs in its own thread
mqtt_client.loop_start()

# ----------------------------------------------------
# SERVER STARTUP
# ----------------------------------------------------

if __name__ == "__main__":
    print(f"MQTT → Socket.IO bridge running on {SOCKETIO_PORT}")
    eventlet.wsgi.server(eventlet.listen(("", SOCKETIO_PORT)), app)
