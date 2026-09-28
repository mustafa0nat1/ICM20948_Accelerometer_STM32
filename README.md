# Accelerometer — 6-Position Calibration

Standalone ICM-20948 accelerometer example for STM32F303K8.

## What it does

- Verifies ICM-20948 with `WHO_AM_I = 0xEA`
- Runs the accelerometer while the gyro is disabled for this standalone test
- Captures six static orientations: +X, -X, +Y, -Y, +Z, -Z
- Computes offset and per-axis scale
- Converts corrected samples to g and m/s²

Calibration model:

```text
offset = (+1g + -1g) / 2
measured_lsb_per_g = (+1g - -1g) / 2
scale = 16384 / measured_lsb_per_g
corrected_raw = (raw - offset) * scale
```

Keep the board still during each capture.
