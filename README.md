This repository contains the implementation of a PID-based Line Following Robot using a 5-sensor IR array and the TB6612FNG motor driver. The robot is designed to follow a black (or white) line with high accuracy, handle turns, and maintain stability using real-time feedback control.

🔧 Features
✅ PID Control (Proportional + Derivative + Integral ready)
✅ 5-Sensor IR Array for precise line detection
✅ Dynamic speed control with smooth acceleration
✅ Automatic calibration of sensors
✅ Sharp turn detection (left/right edge sensing)
✅ Line recovery when path is lost
✅ Supports both black-line and white-line tracking

🧠 How It Works
The robot reads values from 5 IR sensors.
Each sensor is assigned a weight to calculate positional error.
A PID controller computes correction based on deviation from the line.
Motor speeds are adjusted dynamically to keep the robot centered.
Edge sensors detect intersections or sharp turns and trigger corrective rotation.

⚙️ Hardware Used
Arduino (Uno/Nano)
TB6612FNG Motor Driver
5-channel IR Sensor Array
DC Motors with wheels
Chassis + battery setup
Push buttons (for calibration & start)
LED indicator

| Function      | Pin |
| ------------- | --- |
| Motor A IN1   | 4   |
| Motor A IN2   | 3   |
| Motor B IN1   | 6   |
| Motor B IN2   | 7   |
| PWM A         | 9   |
| PWM B         | 10  |
| Start Button  | 11  |
| Run Button    | 12  |
| LED Indicator | 13  |
