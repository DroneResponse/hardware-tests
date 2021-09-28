# Hardware Tests

This runs hardware tests.

## How to run tests

First, start `mavros`:

```bash
roslaunch dr_hardware_tests mavros.launch
```

Next run the hardware tests

```bash
rosrun dr_hardware_tests indoor_sensors.py
rosrun dr_hardware_tests sensors.py
rosrun dr_hardware_tests arm.py
rosrun dr_hardware_tests hover.py

```

TODO explain how to connect it to a serial device.
