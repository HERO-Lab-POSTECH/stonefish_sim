# stonefish_teleop_manager

Teleoperation interface for underwater vehicles (Planned for future development).

## Status

⚠️ **This package is currently a placeholder and not yet implemented.**

## Planned Features

- **Joystick/Gamepad support**: Control vehicle with game controller
- **Keyboard teleoperation**: Simple keyboard-based control
- **Web interface**: Browser-based remote control
- **Safety features**: Deadman switch, velocity limits, geofencing
- **Mode switching**: Toggle between teleoperation and autonomous modes

## Roadmap

1. **Phase 1**: Basic keyboard teleoperation
2. **Phase 2**: Joystick/gamepad support (ROS2 joy package integration)
3. **Phase 3**: Web interface with video streaming
4. **Phase 4**: Advanced features (haptic feedback, VR support)

## Alternatives

Until this package is implemented, you can use existing ROS2 teleoperation tools:

### 1. teleop_twist_keyboard

Keyboard control for ROS2 vehicles:

```bash
sudo apt install ros-humble-teleop-twist-keyboard

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap /cmd_vel:=/bluerov2/cmd_vel
```

### 2. joy (Gamepad/Joystick)

```bash
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy

# Terminal 1: Joy node
ros2 run joy joy_node

# Terminal 2: Twist publisher
ros2 run teleop_twist_joy teleop_node \
    --ros-args --remap /cmd_vel:=/bluerov2/cmd_vel
```

### 3. Custom Script

Create a simple teleoperation node:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class SimpleTeleop(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        self.publisher = self.create_publisher(Twist, '/bluerov2/cmd_vel', 10)

    def get_key(self):
        # Get keyboard input
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        twist = Twist()
        while True:
            key = self.get_key()
            if key == 'w': twist.linear.x = 1.0
            elif key == 's': twist.linear.x = -1.0
            elif key == 'a': twist.angular.z = 1.0
            elif key == 'd': twist.angular.z = -1.0
            elif key == ' ': twist = Twist()  # Stop
            elif key == 'q': break

            self.publisher.publish(twist)

def main():
    rclpy.init()
    node = SimpleTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Contributing

If you would like to contribute to this package, please:

1. Fork the repository
2. Implement teleoperation features
3. Submit a pull request with:
   - Working code
   - Tests
   - Documentation
   - Example launch files

## Contact

For questions or feature requests, please open an issue on GitHub.

## License

GPL-3.0
