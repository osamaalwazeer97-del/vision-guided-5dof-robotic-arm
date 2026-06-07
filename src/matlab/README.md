# MATLAB Code

This folder contains the MATLAB code extracted and cleaned from Appendix A of the graduation project report.

## Files

- `main_vision_sorting.m`: Main vision-guided sorting script.
- `findObjType.m`: Shape/color classification helper.
- `CallPlaceAng.m`: Storage-bin joint command helper.

## Before running

Update these variables in `main_vision_sorting.m`:

```matlab
serialPort      = 'COM7';
i2cAddress      = '0x08';
calibrationFile = 'F:\AAAAA\calibration\matlab.mat';
cameraName      = 'Sirius USB2.0 Camera';
```

Also replace the placeholder bin commands in `CallPlaceAng.m` with your real calibrated bin joint angles.
