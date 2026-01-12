# MP-400 Footprint Switcher

Dynamic footprint switching package for Neobotix MP-400 robot in ROS2.

## Overview

This ROS2 package provides a simple yet powerful solution for dynamically switching the footprint configuration of the Neobotix MP-400 robot between two predefined states:

- **Original**: 0.30m × 0.30m (60cm × 60cm)
- **Extended**: 1.00m × 1.00m (100cm × 100cm)

The footprint affects how the navigation stack (Nav2) calculates collision-free paths, so switching between configurations allows the robot to navigate in different scenarios:
- **Original**: Tight spaces, narrow corridors, precision navigation
- **Extended**: Larger areas, conservative collision margins, safety-focused navigation

## Features

✅ **Dynamic Footprint Switching** - Change footprint without restarting
✅ **Service-Based Control** - Call `/switch_footprint` service to toggle
✅ **Parameter-Based Control** - Set initial footprint at launch time
✅ **Namespace Support** - Works with single or multi-robot setups
✅ **Status Publishing** - Real-time status updates on footprint changes
✅ **Nav2 Integration** - Automatic parameter updates to navigation stack
✅ **Verbose Logging** - Optional detailed logging of all operations

## Installation

### 1. Clone into your ROS2 workspace

```bash
cd ~/ros2_workspace/src
# Copy the mp400_footprint_switcher directory here
cp -r /path/to/mp400_footprint_switcher .
```

### 2. Build the package

```bash
cd ~/ros2_workspace
colcon build --packages-select mp400_footprint_switcher --symlink-install
source install/setup.bash
```

## Usage

### Launch with Default (Original Footprint)

```bash
ros2 launch mp400_footprint_switcher footprint_switcher.launch.py
```

### Launch with Extended Footprint

```bash
ros2 launch mp400_footprint_switcher footprint_switcher.launch.py \
  initial_footprint:=extended
```

### Launch with Robot Namespace

```bash
ros2 launch mp400_footprint_switcher footprint_switcher.launch.py \
  robot_namespace:=robot1 \
  initial_footprint:=original
```

## ROS2 Interface

### Service

**Service Name**: `/mp400/switch_footprint` (or `/{robot_namespace}/switch_footprint`)

**Service Type**: `std_srvs/Trigger`

**Description**: Toggles the footprint between original and extended configurations.

**Example Call**:

```bash
ros2 service call /mp400/switch_footprint std_srvs/srv/Trigger
```

**Response**:
```
result: 
  success: true
  message: "✅ Footprint switched to extended"
```

### Publishers

#### 1. Footprint Status

**Topic**: `/{robot_namespace}/footprint_status`

**Message Type**: `std_msgs/String` (JSON encoded)

**Example Payload**:
```json
{
  "footprint": "extended",
  "points": [[0.50, 0.50], [-0.50, 0.50], [-0.50, -0.50], [0.50, -0.50]],
  "width": 1.0,
  "height": 1.0,
  "timestamp": 1704283200000000000
}
```

#### 2. Local Costmap Footprint

**Topic**: `/{robot_namespace}/local_costmap/footprint`

**Message Type**: `geometry_msgs/Polygon`

#### 3. Global Costmap Footprint

**Topic**: `/{robot_namespace}/global_costmap/footprint`

**Message Type**: `geometry_msgs/Polygon`

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_namespace` | string | `/` | Robot namespace (e.g., "robot1") |
| `initial_footprint` | string | `original` | Initial footprint: "original" or "extended" |
| `update_frequency` | float | `1.0` | Update frequency in Hz |
| `verbose` | bool | `True` | Enable verbose logging |

## Example Use Cases

### Single Robot with Dynamic Navigation

```bash
# Terminal 1: Launch MP-400 robot
ros2 launch neo_mp_400-2 bringup.launch.py robot_namespace:=robot1

# Terminal 2: Launch navigation
ros2 launch neo_mp_400-2 navigation.launch.py robot_namespace:=robot1

# Terminal 3: Launch footprint switcher
ros2 launch mp400_footprint_switcher footprint_switcher.launch.py \
  robot_namespace:=robot1 \
  initial_footprint:=original

# Terminal 4: Switch footprint when needed
ros2 service call /robot1/switch_footprint std_srvs/srv/Trigger
```

### Multi-Robot Setup

```bash
# Robot 1 with original footprint
ros2 launch mp400_footprint_switcher footprint_switcher.launch.py \
  robot_namespace:=robot1 \
  initial_footprint:=original

# Robot 2 with extended footprint
ros2 launch mp400_footprint_switcher footprint_switcher.launch.py \
  robot_namespace:=robot2 \
  initial_footprint:=extended
```

## Python Client Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class FootprintSwitchClient(Node):
    def __init__(self):
        super().__init__('footprint_switch_client')
        self.cli = self.create_client(Trigger, '/mp400/switch_footprint')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for switch_footprint service...')
    
    def switch_footprint(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Response: {response.message}")
            return response.success
        else:
            self.get_logger().error("Service call failed")
            return False

def main(args=None):
    rclpy.init(args=args)
    client = FootprintSwitchClient()
    
    # Switch footprint
    success = client.switch_footprint()
    
    if success:
        print("✅ Footprint switched successfully!")
    else:
        print("❌ Footprint switch failed!")
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Monitoring Footprint Changes

### Monitor Status Topic

```bash
# Watch footprint status changes
ros2 topic echo /mp400/footprint_status
```

### Monitor Costmap Footprints

```bash
# Local costmap footprint
ros2 topic echo /robot1/local_costmap/footprint

# Global costmap footprint
ros2 topic echo /robot1/global_costmap/footprint
```

## Troubleshooting

### Service Not Available

```bash
# Check if the node is running
ros2 node list | grep footprint_switcher

# Check service availability
ros2 service list | grep switch_footprint

# Try launching with verbose output
ros2 launch mp400_footprint_switcher footprint_switcher.launch.py verbose:=True
```

### Footprint Not Updating in Nav2

1. Ensure Nav2 is running
2. Check that your Nav2 parameters reference the correct topics
3. Verify the robot namespace matches between switcher and Nav2

```bash
# Check which parameters Nav2 is using
ros2 param list /bt_navigator
ros2 param get /bt_navigator local_costmap
```

## Architecture

```
┌─────────────────────────────────────────────────────┐
│     MP-400 Footprint Switcher Node                  │
├─────────────────────────────────────────────────────┤
│                                                     │
│  ┌─────────────────┐         ┌──────────────────┐  │
│  │  Service Server │         │ Parameter Server │  │
│  │  (switch_foot)  │         │   (get/set)      │  │
│  └────────┬────────┘         └────────┬─────────┘  │
│           │                           │             │
│           └───────────┬───────────────┘             │
│                       │                             │
│           ┌───────────▼──────────┐                  │
│           │ Footprint Application│                  │
│           │   Engine             │                  │
│           └───────────┬──────────┘                  │
│                       │                             │
│       ┌───────────────┼───────────────┐             │
│       │               │               │             │
│  ┌────▼────┐    ┌─────▼─────┐   ┌────▼────┐        │
│  │Local CM  │    │ Global CM │   │ Status  │        │
│  │Footprint │    │ Footprint │   │ Topic   │        │
│  │Publisher │    │ Publisher │   │Publisher│        │
│  └──────────┘    └───────────┘   └────────┘        │
│                                                     │
└─────────────────────────────────────────────────────┘
         │                    │              │
         │                    │              │
         ▼                    ▼              ▼
    Local Costmap       Global Costmap    ROS Topics
```

## License

Apache 2.0 - See LICENSE file

## Contributing

Contributions are welcome! Please submit pull requests or issues to the repository.

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review the example scripts
3. Check ROS2 logs: `ros2 launch mp400_footprint_switcher footprint_switcher.launch.py verbose:=True`
