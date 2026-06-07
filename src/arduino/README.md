# Arduino Code

This folder contains the Arduino Mega controller code extracted and cleaned from Appendix B of the graduation project report.

## Main file

- `robot_arm_controller/robot_arm_controller.ino`

## Required libraries

Install these in Arduino IDE:

- `PinChangeInterrupt`
- `digitalPinFast`
- `Servo`
- `Wire`

## Notes

The Arduino acts as I2C slave address `8`. MATLAB sends an 8-element vector:

```text
[q1 q2 q3 pitch roll gripper enablePower resetInitial]
```

The first three joint angles are converted to encoder slots using approximately `2.5 slots/degree`, consistent with the 0.4 degree encoder resolution described in the report.
