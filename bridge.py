import paho.mqtt.client as mqtt
import time
import re
import sys
from typing import NamedTuple, Optional, Union
from influxdb import InfluxDBClient
import json
from datetime import datetime

# InfluxDB configuration
INFLUXDB_ADDRESS = ''
INFLUXDB_USER = ''
INFLUXDB_PASSWORD = ''
INFLUXDB_DATABASE = 'Robot'

# MQTT settings
MQTT_BROKER = ""
MQTT_USER = ''
MQTT_PASSWORD = ''
MQTT_TOPIC_BAT = "esp32/batterij"
MQTT_TOPIC_ULTRA = "esp32/ultrasone"
MQTT_TOPIC_STATUS = "esp32/status"
CLIENT_ID = "mqttID"
MQTT_REGEX = r'(raspberry_pi|esp32)/([^/]+)(?:/([^/]+))?'
MQTT_CLIENT_ID = 'MQTTInfluxDBEindwerk'

influxdb_client = InfluxDBClient(INFLUXDB_ADDRESS, 8086, INFLUXDB_USER, INFLUXDB_PASSWORD, None)

class SensorData(NamedTuple):
    location: str
    measurement: str
    value: Union[float, str]  # Can now be text or numeric
    timestamp: Optional[str] = None

def init_influxdb():
    """Ensure the target database exists and switch to it."""
    try:
        databases = influxdb_client.get_list_database()
        if not any(db['name'] == INFLUXDB_DATABASE for db in databases):
            print(f"Creating database: {INFLUXDB_DATABASE}")
            influxdb_client.create_database(INFLUXDB_DATABASE)
        influxdb_client.switch_database(INFLUXDB_DATABASE)
        print(f"Connected to InfluxDB database: {INFLUXDB_DATABASE}")
    except Exception as e:
        print(f"Error initializing InfluxDB: {e}")
        sys.exit(1)

def on_connect(client, userdata, flags, rc, properties=None):
    """Callback for MQTT connection"""
    if rc == 0:
        print(f"Connected to MQTT broker (code: {rc})")
        # Subscribe to all robot topics
        client.subscribe(MQTT_TOPIC_ULTRA)
        client.subscribe(MQTT_TOPIC_BAT)
        client.subscribe(MQTT_TOPIC_STATUS)
        print("Subscribed to robot topics")
    else:
        print(f"Failed to connect to MQTT broker (code: {rc})")

def parse_mqtt_message(topic, payload):
    """Parse the topic and payload and return sensor data"""
    try:
        # Extract device and measurement from topic
        parts = topic.split('/')
        if len(parts) >= 2:
            device = parts[0]  # esp32
            measurement = parts[1]  # batterij, ultrasone, status
            
            # Handle different message types
            if measurement == "batterij":
                # Battery percentage
                try:
                    value = float(payload)
                    return SensorData(
                        location=device,
                        measurement="battery_percentage",
                        value=value
                    )
                except ValueError:
                    print(f"Could not parse battery value: {payload}")
                    return None
                    
            elif measurement == "ultrasone":
                # Ultrasonic sensor - could be distance or status message
                try:
                    # Try to parse as numeric distance first
                    value = float(payload)
                    return SensorData(
                        location=device,
                        measurement="distance_cm",
                        value=value
                    )
                except ValueError:
                    # Handle status messages - keep as text
                    return SensorData(
                        location=device,
                        measurement="ultrasonic_status",
                        value=payload  # Keep original text
                    )
                        
            elif measurement == "status":
                # Robot status messages - keep as original text
                return SensorData(
                    location=device,
                    measurement="robot_status",
                    value=payload  # Keep original text instead of converting to numbers
                )
                
        return None
        
    except Exception as e:
        print(f"Error parsing MQTT message: {e}")
        return None

def send_to_influxdb(sensor_data):
    """Format the sensor data and write it to InfluxDB."""
    try:
        # Determine field type based on value type
        if isinstance(sensor_data.value, str):
            # For text values, use a string field
            fields = {"text_value": sensor_data.value}
        else:
            # For numeric values, use a numeric field
            fields = {"value": sensor_data.value}
        
        json_body = [
            {
                "measurement": sensor_data.measurement,
                "tags": {
                    "location": sensor_data.location,
                    "device": "line_following_robot"
                },
                "time": datetime.utcnow().isoformat() + "Z",
                "fields": fields
            }
        ]
        
        influxdb_client.write_points(json_body)
        print(f"Sent to InfluxDB: {sensor_data.measurement} = {sensor_data.value}")
        
    except Exception as e:
        print(f"Error writing to InfluxDB: {e}")

def on_message(client, userdata, message):
    """Callback for MQTT message reception"""
    try:
        topic = message.topic
        payload = message.payload.decode('utf-8')
        
        print(f"Received: {topic} -> {payload}")
        
        # Parse the message
        sensor_data = parse_mqtt_message(topic, payload)
        
        if sensor_data:
            # Send to InfluxDB
            send_to_influxdb(sensor_data)
        else:
            print(f"Could not parse message from topic: {topic}")
            
    except Exception as e:
        print(f"Error processing MQTT message: {e}")

def main():
    """Main function to run the MQTT to InfluxDB bridge"""
    print("Starting MQTT to InfluxDB bridge for robot dashboard...")
    
    # Initialize InfluxDB
    init_influxdb()
    
    # Setup MQTT client
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, CLIENT_ID)
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    
    # Connect to MQTT broker
    try:
        print(f"Connecting to MQTT broker: {MQTT_BROKER}")
        mqtt_client.connect(MQTT_BROKER, 1883, 60)
    except Exception as e:
        print(f"Cannot connect to MQTT broker: {e}")
        sys.exit(1)
    
    # Start MQTT loop
    mqtt_client.loop_start()
    
    print("Bridge is running. Data will be sent to InfluxDB for Grafana visualization.")
    print("Monitoring the following topics:")
    print(f"  - {MQTT_TOPIC_BAT} (Battery percentage)")
    print(f"  - {MQTT_TOPIC_ULTRA} (Distance sensor)")
    print(f"  - {MQTT_TOPIC_STATUS} (Robot status)")
    print("\nPress Enter to stop...")
    
    try:
        input()
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    print("Bridge stopped.")

if __name__ == "__main__":
    main()
