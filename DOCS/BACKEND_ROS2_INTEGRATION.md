# Backend-ROS2 Integration Guide

## Overview

The warebot system now includes complete Flask-ROS2 integration for autonomous robot task navigation. This guide covers setup, usage, and architecture.

---

## Architecture Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    Frontend (React/Vite)                        │
│  - Map visualization with real-time robot updates             │
│  - Task assignment UI                                         │
│  - WebSocket events: robot_status, task_status                │
└─────────────────────────────────────────────────────────────────┘
                              │
                    (WebSocket/HTTP)
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                  Backend (Flask/SocketIO)                       │
│  - REST API: POST /api/tasks/assign                           │
│  - WebSocket: emit task_status events                         │
│  - ROS2IntegrationService: bridges to ROS2                    │
└─────────────────────────────────────────────────────────────────┘
                              │
                        (MQTT)
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                  ROS2 Task Navigator System                      │
│  ┌────────────────────────────────────────────────────────┐   │
│  │ 1. TaskNavigator Node                                  │   │
│  │    - Receives task assignments from backend            │   │
│  │    - State machine: MOVING → AT_TARGET → RETURNING    │   │
│  │    - Publishes velocity commands                       │   │
│  │    - Manages robot navigation                          │   │
│  └────────────────────────────────────────────────────────┘   │
│  ┌────────────────────────────────────────────────────────┐   │
│  │ 2. TaskExecutor Node                                   │   │
│  │    - Executes velocity commands on individual robots   │   │
│  │    - Simulates robot kinematics (for testing)         │   │
│  │    - Publishes odometry/pose updates                  │   │
│  │    - Sends movement status                            │   │
│  └────────────────────────────────────────────────────────┘   │
│  ┌────────────────────────────────────────────────────────┐   │
│  │ 3. ReferencePointManager Node                          │   │
│  │    - Stores robot home/dock locations                 │   │
│  │    - Manages reference points (create/update/delete)  │   │
│  │    - Enables autonomous return navigation             │   │
│  └────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                        (MQTT/ROS2)
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    Robot Hardware                               │
│  - Receive velocity commands                                  │
│  - Publish odometry/pose telemetry                            │
│  - Execute navigation tasks                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## MQTT Topics Reference

### Task Assignment Topics

**Backend → ROS2:**
- `ros2/task/assignment` - Task assignment JSON
  ```json
  {
    "task_id": "task_robot_1_1700000000000",
    "robot_id": "robot_1",
    "target_x": 5.2,
    "target_y": 3.8,
    "target_yaw": 0.0,
    "priority": 1,
    "timestamp": 1700000000.123
  }
  ```

### Reference Point Topics

**Backend → ROS2:**
- `ros2/reference_point/command` - Reference point commands
  ```json
  {
    "command": "set",
    "robot_id": "robot_1",
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0,
    "description": "Main Dock"
  }
  ```

### Robot Control Topics

**Backend → ROS2:**
- `ros2/robot/{robot_id}/command` - Robot command (start/stop)
  ```json
  {
    "action": "start",
    "task_id": "task_robot_1_1700000000000"
  }
  ```

### ROS2 Internal Topics

**Navigation:**
- `/task/assignment` - Task assignment (JSON string)
- `/robot/pose` - Robot position [x, y, yaw]
- `/reference_point/update` - Reference point data
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/task/status` - Current task status
- `/robot/state` - Robot state (IDLE, MOVING_TO_TARGET, etc.)

**Execution:**
- `/odometry` - Odometry updates [x, y, yaw]
- `/movement/status` - Movement execution status

---

## Setup Instructions

### 1. Prerequisites

```bash
# Install ROS2 Jazzy
sudo apt install ros-jazzy-desktop

# Install Python dependencies
pip install paho-mqtt rclpy geometry-msgs

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash
```

### 2. Build ROS2 Package

```bash
cd ~/jazzy_ws
colcon build --packages-select warebot_task_navigator
source install/setup.bash
```

### 3. Start Backend

```bash
cd /home/super/Desktop/warebot-backend/backend
python main.py
# Backend runs on http://localhost:5000
```

### 4. Launch ROS2 Nodes

```bash
# Terminal 1: Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/jazzy_ws/install/setup.bash

# Terminal 2: Launch task navigator system
ros2 launch warebot_task_navigator task_navigator.launch.py \
  mqtt_broker:=localhost \
  mqtt_port:=1883 \
  robot_id:=robot_1
```

### 5. Optional: Monitor ROS2 Topics

```bash
# In separate terminals, monitor different topics:

# Task assignments
ros2 topic echo /task/assignment

# Robot pose
ros2 topic echo /robot/pose

# Velocity commands
ros2 topic echo /cmd_vel

# Task status
ros2 topic echo /task/status

# Reference points
ros2 topic echo /reference_point/update
```

---

## API Usage Examples

### 1. Assign Task to Robot

```bash
curl -X POST http://localhost:5000/api/tasks/assign \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -d '{
    "robot_id": "robot_1",
    "target_x": 5.2,
    "target_y": 3.8,
    "target_yaw": 0.0,
    "priority": 1,
    "reference_point_x": 0.0,
    "reference_point_y": 0.0,
    "reference_point_yaw": 0.0
  }'
```

**Response:**
```json
{
  "task_id": "task_robot_1_1700000000000",
  "robot_id": "robot_1",
  "target_x": 5.2,
  "target_y": 3.8,
  "target_yaw": 0.0,
  "priority": 1,
  "status": "ASSIGNED",
  "message": "Task assigned to robot"
}
```

### 2. Set Reference Point (Home/Dock)

```bash
curl -X POST http://localhost:5000/api/tasks/robot_1/reference-point \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -d '{
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0,
    "description": "Main Dock"
  }'
```

**Response:**
```json
{
  "robot_id": "robot_1",
  "x": 0.0,
  "y": 0.0,
  "yaw": 0.0,
  "description": "Main Dock",
  "status": "SUCCESS",
  "message": "Reference point updated"
}
```

### 3. Start Task Execution

```bash
curl -X POST http://localhost:5000/api/tasks/robot_1/start \
  -H "Authorization: Bearer YOUR_JWT_TOKEN"
```

**Response:**
```json
{
  "robot_id": "robot_1",
  "status": "EXECUTING",
  "message": "Task execution started"
}
```

### 4. Stop Task Execution

```bash
curl -X POST http://localhost:5000/api/tasks/robot_1/stop \
  -H "Authorization: Bearer YOUR_JWT_TOKEN"
```

**Response:**
```json
{
  "robot_id": "robot_1",
  "status": "STOPPED",
  "message": "Task execution stopped"
}
```

---

## Robot Navigation State Machine

```
┌─────────────────────────────────────────────────────────┐
│                  Task Assignment Received                │
│                    (PENDING state)                       │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                   Validate Task Data                     │
│          - Check target coordinates                     │
│          - Set up reference point                       │
│          - Initialize movement parameters               │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                   MOVING_TO_TARGET                       │
│  - Calculate direction to target                        │
│  - Send velocity commands (Twist)                       │
│  - Update robot position from odometry                  │
│  - Check if within tolerance (0.1m, 0.1rad)            │
│  - Exit when distance < tolerance                       │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                     AT_TARGET                            │
│  - Stop velocity commands                               │
│  - Update task status to COMPLETED                      │
│  - Prepare for return navigation                        │
│  - Set reference point as next target                   │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                  RETURNING_TO_HOME                       │
│  - Calculate direction to reference point               │
│  - Send velocity commands (Twist)                       │
│  - Update robot position from odometry                  │
│  - Check if within tolerance (0.1m, 0.1rad)            │
│  - Exit when distance < tolerance                       │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                      AT_HOME                             │
│  - Stop velocity commands                               │
│  - Update robot status to IDLE                          │
│  - Ready for new task assignment                        │
│  - Store task completion in database                    │
└─────────────────────────────────────────────────────────┘
```

---

## WebSocket Events

The frontend receives real-time updates via WebSocket:

### 1. Robot Status Update

```javascript
socket.on('robot_status', (data) => {
  console.log('Robot position:', {
    robot_id: data.robot_id,
    x: data.x,
    y: data.y,
    yaw: data.yaw,
    status: data.status,  // IDLE, MOVING_TO_TARGET, AT_TARGET, etc.
    battery: data.battery_level
  });
});
```

### 2. Task Status Update

```javascript
socket.on('task_status', (data) => {
  console.log('Task update:', {
    task_id: data.task_id,
    robot_id: data.robot_id,
    status: data.status,  // PENDING, ASSIGNED, IN_PROGRESS, COMPLETED
    progress: data.progress,  // 0-100
    target_x: data.target_x,
    target_y: data.target_y
  });
});
```

### 3. Error Notification

```javascript
socket.on('error', (data) => {
  console.error('System error:', {
    error_type: data.error_type,
    message: data.message,
    robot_id: data.robot_id,
    timestamp: data.timestamp
  });
});
```

---

## Testing Workflow

### Step 1: Verify MQTT Connection
```bash
# Check if MQTT broker is running
mosquitto_sub -h localhost -p 1883 -t "ros2/#"
```

### Step 2: Monitor ROS2 Topics
```bash
# In separate terminals
ros2 topic list
ros2 topic echo /task/assignment
ros2 topic echo /robot/pose
```

### Step 3: Test Task Assignment
```bash
# Use the curl examples above to assign a task
# Monitor the MQTT and ROS2 topic output
```

### Step 4: Verify State Transitions
```bash
# Watch the robot state changes
ros2 topic echo /robot/state

# Expected sequence:
# IDLE → MOVING_TO_TARGET → AT_TARGET → RETURNING_TO_HOME → AT_HOME → IDLE
```

### Step 5: Check WebSocket Events
```bash
# In browser console, connected to frontend:
socket.on('task_status', (data) => console.log('Task:', data));
socket.on('robot_status', (data) => console.log('Robot:', data));
```

---

## Troubleshooting

### Issue: ROS2 nodes don't start

**Solution:**
```bash
# Ensure ROS2 setup is sourced
source /opt/ros/jazzy/setup.bash
source ~/jazzy_ws/install/setup.bash

# Check if package is built
colcon list | grep warebot_task_navigator

# Rebuild if needed
cd ~/jazzy_ws
colcon build --packages-select warebot_task_navigator
```

### Issue: MQTT connection fails

**Solution:**
```bash
# Check MQTT broker status
paho-mqtt-test localhost 1883

# Or use mosquitto tools
mosquitto_pub -h localhost -p 1883 -t "test" -m "hello"
mosquitto_sub -h localhost -p 1883 -t "test"
```

### Issue: Backend fails to find ros2_integration service

**Solution:**
```bash
# Ensure app/__init__.py has the import
grep "get_ros2_service" backend/app/__init__.py

# Verify services/__init__.py exports it
grep "ros2_integration" backend/app/services/__init__.py
```

### Issue: Tasks not being published to MQTT

**Solution:**
1. Check MQTT client is initialized in app
2. Verify ros2_integration.set_mqtt_client() is called
3. Monitor MQTT topics: `mosquitto_sub -h localhost -p 1883 -t "ros2/#"`

---

## Performance Tuning

### Control Loop Frequency
- Default: 10Hz (100ms per cycle)
- Adjust in launch file: `control_frequency: 10.0`
- Higher = more responsive, more CPU usage

### Navigation Tolerances
- Distance tolerance: 0.1m (default)
- Angle tolerance: 0.1 rad (default)
- Adjust in launch file for precision needs

### Speed Limits
- Max linear speed: 0.5 m/s
- Max angular speed: 0.5 rad/s
- Adjust for safety requirements

---

## Future Enhancements

1. **Multi-task Queue:** Support task queuing per robot
2. **Collision Avoidance:** Add obstacle detection
3. **Path Planning:** Implement A* or RRT algorithms
4. **Dynamic Reference Points:** Update docks on-the-fly
5. **GPS Integration:** Use GPS for outdoor navigation
6. **Battery Management:** Return to dock when battery low
7. **Task Priority Queue:** Execute higher priority tasks first

---

## Files Modified/Created

```
backend/
├── app/
│   ├── __init__.py (updated: added ros2_integration setup)
│   ├── routes/
│   │   └── api_task_assignment.py (created: task API endpoints)
│   ├── services/
│   │   ├── __init__.py (updated: export ros2_integration)
│   │   └── ros2_integration.py (created: Flask-ROS2 bridge)

warebot_task_navigator/
├── launch/
│   └── task_navigator.launch.py (created: launch configuration)
├── warebot_task_navigator/
│   ├── task_navigator.py (node: main coordinator)
│   ├── task_executor.py (node: velocity execution)
│   ├── reference_point_manager.py (node: home/dock management)
```

---

## Architecture Decisions

### Why MQTT?
- **Decoupling:** Loosely couples Flask and ROS2
- **Reliability:** Message queuing for offline tolerance
- **Scalability:** Easy to add multiple robots/services
- **Standard:** Industry-standard protocol for IoT

### Why State Machine?
- **Clarity:** Explicit navigation states
- **Safety:** Prevents invalid transitions
- **Debugging:** Easy to track robot state history
- **Testing:** Deterministic behavior

### Why Separate Nodes?
- **TaskNavigator:** Orchestration logic (10Hz planning)
- **TaskExecutor:** Execution layer (10Hz control)
- **ReferencePointManager:** Configuration management (1Hz)
- **Separation of concerns:** Each node has one responsibility

---

**Last Updated:** November 28, 2025  
**Version:** 1.0 - Production Ready
