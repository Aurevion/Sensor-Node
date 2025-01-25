# Sensor-Node

This ROS2 package, `sensor_node`, implements a node that communicates with a sensor via TCP. The node decodes incoming status messages and publishes the extracted parameters to specific ROS2 topics. Below are the details of the decoding process, the parameters published, and their respective topic names and data types.

---

## **Decoding Status Messages**

The sensor sends status messages in a predefined format. Each message contains the following:

- **Start Character (`$`)**: Indicates the beginning of a message.
- **Command ID (`11`)**: Identifies the message as a status message.
- **Payload (10 bytes)**: Encodes various sensor parameters.
- **End Characters (`\r\n`)**: Indicates the end of the message.

The payload contains the following parameters, encoded in Little-Endian format:

| Parameter          | Type     | Size  | Unit             | Description                        |
|--------------------|----------|-------|------------------|------------------------------------|
| Supply Voltage     | `uint16` | 2 bytes | Millivolts        | Voltage supplied to the sensor.    |
| Environmental Temp | `int16`  | 2 bytes | Deci-Celsius      | Sensor's temperature.              |
| Yaw                | `int16`  | 2 bytes | Deci-Degrees      | Current yaw angle of the sensor.   |
| Pitch              | `int16`  | 2 bytes | Deci-Degrees      | Current pitch angle of the sensor. |
| Roll               | `int16`  | 2 bytes | Deci-Degrees      | Current roll angle of the sensor.  |

### **Example Message Decoding**

A status message with payload `0A1E0A320032003200` is decoded as:
- **Supply Voltage**: `0A1E` = `2590 mV` (in millivolts)
- **Environmental Temp**: `0A32` = `810 dC` (or 81.0 degrees Celsius)
- **Yaw**: `0032` = `50.0 degrees`
- **Pitch**: `0032` = `50.0 degrees`
- **Roll**: `0032` = `50.0 degrees`

---

## **Published Parameters**

The decoded parameters are published to the following ROS2 topics:

| **Parameter**        | **Topic Name**     | **Message Type** | **Unit**         |
|----------------------|--------------------|------------------|------------------|
| Supply Voltage       | `supply_voltage`  | `std_msgs/Float32` | Volts (V)       |
| Environmental Temp   | `env_temp`        | `std_msgs/Float32` | Celsius (°C)    |
| Yaw                  | `yaw`             | `std_msgs/Float32` | Degrees (°)     |
| Pitch                | `pitch`           | `std_msgs/Float32` | Degrees (°)     |
| Roll                 | `roll`            | `std_msgs/Float32` | Degrees (°)     |

---

## **Node Behavior**

1. **Start Command**: Sends a start command to the sensor with a configurable interval (in milliseconds).
2. **Receive and Decode Messages**: Listens for status messages from the sensor, decodes them, and publishes the parameters.
3. **Services**:
   - `start_sensor`: Start the sensor with a custom interval.
   - `stop_sensor`: Stop the sensor from sending status messages.

---

## **Start and Stop the Sensor**

The sensor node provides two services, `start_sensor` and `stop_sensor`, to allow users to control the sensor's behavior.

### **Start the Sensor with a Custom Interval**
The `start_sensor` service allows users to specify a custom interval (in milliseconds) for the sensor to send status messages. To start the sensor:

1. Call the `start_sensor` service:
   ```bash
   ros2 service call /start_sensor sensor_node/srv/StartSensor "{interval: 1000}"
   ```
   Replace `1000` with your desired interval in milliseconds.

2. The node will send a start command to the sensor and the interval will be updated.

### **Stop the Sensor**
The `stop_sensor` service stops the sensor from sending status messages. To stop the sensor:

1. Call the `stop_sensor` service:
   ```bash
   ros2 service call /stop_sensor sensor_node/srv/StopSensor "{}"
   ```

2. The node will send a stop command to the sensor.

---
