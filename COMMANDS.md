# Roomba ROS2 Control - Command Reference

This document provides command examples for controlling your Roomba via ROS2 topics.

---

## Table of Contents

- [LED Control](#led-control)
- [Motor Control](#motor-control)
- [Mode Control](#mode-control)
- [Sensor Reading](#sensor-reading)

---

## LED Control

Control the Roomba's LEDs using the `gpio_command_controller`.

### Message Format

```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{
  interface_groups: ["led_bits", "led_color", "led_intensity"],
  interface_values: [
    {interface_names: ["value"], values: [LED_BITS]},
    {interface_names: ["value"], values: [COLOR]},
    {interface_names: ["value"], values: [INTENSITY]}
  ]
}'
```

### Parameters

#### LED Bits (0-63) - Which LEDs to turn on

Bitmask where each bit controls a specific LED:

| Bit | Value | LED |
|-----|-------|-----|
| 0   | 1     | Clean button |
| 1   | 2     | Spot button |
| 2   | 4     | Dock button |
| 3   | 8     | Check Robot |
| 4   | 16    | Scheduling |
| 5   | 32    | Dirt Detect |

**Examples:**
- `1` = Only Clean LED
- `7` = Clean + Spot + Dock (1+2+4)
- `63` = All LEDs on (1+2+4+8+16+32)
- `0` = All LEDs off

#### LED Color (0-255)

Controls the color spectrum:
- `0` = Red
- `128` ≈ Green
- `255` = Blue/Violet

#### LED Intensity (0-255)

Brightness level:
- `0` = Off
- `128` = Medium brightness
- `255` = Maximum brightness

---

### LED Command Examples

#### 🔴 All LEDs ON, red, full brightness
```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["led_bits", "led_color", "led_intensity"], interface_values: [{interface_names: ["value"], values: [63]}, {interface_names: ["value"], values: [0]}, {interface_names: ["value"], values: [255]}]}'
```

#### 🟢 Clean button only, green, medium brightness
```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["led_bits", "led_color", "led_intensity"], interface_values: [{interface_names: ["value"], values: [1]}, {interface_names: ["value"], values: [128]}, {interface_names: ["value"], values: [128]}]}'
```

#### 🔵 Dock + Spot buttons, blue, dim
```bash
# Dock(4) + Spot(2) = 6
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["led_bits", "led_color", "led_intensity"], interface_values: [{interface_names: ["value"], values: [6]}, {interface_names: ["value"], values: [255]}, {interface_names: ["value"], values: [64]}]}'
```

#### ⚫ Turn off all LEDs
```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["led_bits", "led_color", "led_intensity"], interface_values: [{interface_names: ["value"], values: [0]}, {interface_names: ["value"], values: [0]}, {interface_names: ["value"], values: [0]}]}'
```

#### 🌈 White-ish, all LEDs, max brightness
```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["led_bits", "led_color", "led_intensity"], interface_values: [{interface_names: ["value"], values: [63]}, {interface_names: ["value"], values: [128]}, {interface_names: ["value"], values: [255]}]}'
```

---

## Motor Control

### Brush Control

#### Turn on main brush (duty_cycle: 0.0-1.0)
```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["main_brush"], interface_values: [{interface_names: ["duty_cycle"], values: [1.0]}]}'
```

#### Turn on side brush
```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["side_brush"], interface_values: [{interface_names: ["duty_cycle"], values: [1.0]}]}'
```

#### Turn on vacuum
```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["vacuum"], interface_values: [{interface_names: ["duty_cycle"], values: [1.0]}]}'
```

#### Turn off all brushes
```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["main_brush", "side_brush", "vacuum"], interface_values: [{interface_names: ["duty_cycle"], values: [0.0]}, {interface_names: ["duty_cycle"], values: [0.0]}, {interface_names: ["duty_cycle"], values: [0.0]}]}'
```

---

## Mode Control

Send Roomba operating mode commands (opcode 143).

### Available Modes

| Value | Mode |
|-------|------|
| 128   | Safe Mode |
| 131   | Full Mode |
| 132   | Passive Mode |
| 133   | Safe Mode (alternative) |
| 134   | Passive Mode (alternative) |
| 135   | Clean Mode |
| 136   | Max Mode |
| 137   | Spot Mode |
| 138   | Seek Dock Mode |

### Send Mode Command

#### Set to Safe Mode (128)
```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["mode"], interface_values: [{interface_names: ["value"], values: [128.0]}]}'
```

#### Set to Clean Mode (135)
```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["mode"], interface_values: [{interface_names: ["value"], values: [135.0]}]}'
```

#### Set to Full Mode (131)
```bash
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["mode"], interface_values: [{interface_names: ["value"], values: [131.0]}]}'
```

---

## Sensor Reading

### Monitor Sensor States

Subscribe to sensor data:

```bash
ros2 topic echo /joint_states
```

Or view specific sensor interfaces:

```bash
# Battery voltage
ros2 topic echo /sensor_data/state_interface/voltage

# Battery charge
ros2 topic echo /sensor_data/state_interface/charge

# Bump sensors
ros2 topic echo /sensor_data/state_interface/bumps_right
ros2 topic echo /sensor_data/state_interface/bumps_left

# Cliff sensors
ros2 topic echo /sensor_data/state_interface/cliff_left
ros2 topic echo /sensor_data/state_interface/cliff_front_left
ros2 topic echo /sensor_data/state_interface/cliff_front_right
ros2 topic echo /sensor_data/state_interface/cliff_right

# Wheel encoders
ros2 topic echo /sensor_data/state_interface/encoder_counts_left
ros2 topic echo /sensor_data/state_interface/encoder_counts_right

# Odometry (calculated)
ros2 topic echo /sensor_data/state_interface/x
ros2 topic echo /sensor_data/state_interface/y
ros2 topic echo /sensor_data/state_interface/theta
```

### View All Sensor Data in RViz

```bash
rviz2 -d src/robot_description/rviz/robot_x.rviz
```

---

## Quick Reference Card

### Common LED Combinations

| Command | Bits | Color | Intensity | Effect |
|---------|------|-------|-----------|--------|
| All ON, white, max | 63 | 128 | 255 | Party mode! |
| Clean only, green | 1 | 128 | 128 | Ready to clean |
| Spot+Dock, orange | 6 | 64 | 200 | Charging station ready |
| Everything OFF | 0 | 0 | 0 | Lights out |

### Common Operations

```bash
# Start cleaning with LEDs on
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["mode"], interface_values: [{interface_names: ["value"], values: [135.0]}]}'
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["led_bits", "led_color", "led_intensity"], interface_values: [{interface_names: ["value"], values: [63]}, {interface_names: ["value"], values: [128]}, {interface_names: ["value"], values: [255]}]}'

# Stop and turn off everything
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["mode"], interface_values: [{interface_names: ["value"], values: [128.0]}]}'
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["led_bits", "led_color", "led_intensity"], interface_values: [{interface_names: ["value"], values: [0]}, {interface_names: ["value"], values: [0]}, {interface_names: ["value"], values: [0]}]}'
ros2 topic pub /gpio_command_controller/commands control_msgs/msg/DynamicInterfaceGroupValues '{interface_groups: ["main_brush", "side_brush", "vacuum"], interface_values: [{interface_names: ["duty_cycle"], values: [0.0]}, {interface_names: ["duty_cycle"], values: [0.0]}, {interface_names: ["duty_cycle"], values: [0.0]}]}'
```

---

## Troubleshooting

### Commands not working?

1. **Check controller is running:**
   ```bash
   ros2 node list | grep gpio_command_controller
   ```

2. **Verify topic exists:**
   ```bash
   ros2 topic info /gpio_command_controller/commands
   ```

3. **Check hardware interface logs:**
   ```bash
   journalctl -u ros2_control -f
   # or view terminal output where launch was started
   ```

4. **Confirm GPIO interfaces are exported:**
   ```bash
   ros2 control list_hardware_interfaces
   ```

### LED colors look wrong?

- Color values are approximate; experiment with different values (0-255)
- Some Roomba models have limited color ranges
- Intensity affects perceived color

---

## Additional Resources

- [Roomba Open Interface Specification](https://www.irobotweb.com/-/media/MainSite/PDFs/About/STEM/Create/create_2_open_interface_spec.pdf)
- [ROS2 control_msgs Documentation](https://docs.ros.org/en/humble/api/control_msgs/html/index.html)
- [gpio_controllers Package](https://github.com/ros-controls/gpio_controllers)
