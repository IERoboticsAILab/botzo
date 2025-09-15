<div align="center">
<h1>BOTZO 🐾</h1>

**`The good boy quadruped robot :)`**

<p align="center">
    <a href="https://www.instagram.com/botzo.ie/" target="_blank" rel="noopener noreferrer">
        <img alt="Instagram" src="https://img.shields.io/badge/Instagram-%232C3454.svg?style=for-the-badge&logo=Instagram&logoColor=white" />
    </a>
    <a href="" target="_blank" rel="noopener noreferrer">
        <img alt="LinkedIn" src="https://img.shields.io/badge/Youtube-%232C3454.svg?style=for-the-badge&logo=Youtube&logoColor=white" />
    </a>
    <a href="mailto:botzoteam@gmail.com">
        <img alt="Gmail" src="https://img.shields.io/badge/Gmail-2c3454?style=for-the-badge&logo=gmail&logoColor=white" />
    </a>
    <a href="">
        <img alt="Views" src="https://komarev.com/ghpvc/?username=botzo&color=blue&style=for-the-badge&abbreviated=true" />
    </a>

</p>

</div>

more [here](https://github.com/IERoboticsAILab/botzo)

# Robot Dog Servo Calibration


This repository contains a Python script designed for calibrating servos for a robot dog application. The calibration process uses Google Colab to map each servo's PWM signal to its actual angle, improving accuracy by correcting for each servo's unique error profile. This guide walks through the setup, usage, and the methodology used to achieve accurate servo positioning.

We have to calibrate each servo motor, because the datasheets are an ideal functionig  of the motor, but in the reality to achive precision we need to adjust the desire angle with some error to actualy reach it. So in this repo we start with the assumptions of the Datasheet and correct the actual values to achive precision.

## Result
You should end up with some results like this:

<p align="center">
  <img src="https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/result_of_one_leg_calibration.png" alt="result" width="400"/>
</p>

---

<p align="center">
    <table>
    <tr>
        <td align="center"><strong>Before Calibration</strong></td>
        <td align="center"><strong>After Calibration</strong></td>
    </tr>
    <tr>
        <td>
        <img src="https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/before_calibration.gif" alt="Before calibration" width="300"/>
        </td>
        <td>
        <img src="https://github.com/IERoboticsAILab/botzo/blob/main/media_assests/after_calibration.gif" alt="After calibration" width="300"/>
        </td>
    </tr>
    </table>
</p>


For higer accuracy!!!

## Repo Content

1. Calibration_Procedure.ipynb: Google collab that will find coefficient for least square regression to convert degrees to PWM with accuracy for each servo

2. calibarte_servos.ino: Arduino code that with help of [Calibration Tool](https://github.com/botzo-team/STL_files/tree/main/servo_calibration_tools) will find real PWM to reach known angles

3. save_coefficent: Folder containg both, .csv and .xlxs, used to save and store calibration values and coefficients for each servo in each leg

## Background and Motivation

Servos are typically controlled by PWM (Pulse Width Modulation) signals to achieve specific angles. However, in practice, each servo may have slightly different mechanical and electrical tolerances, causing deviations between the desired angle and the actual angle. This calibration script was developed to:
1. Map each servo's specific response curve (desired angle vs. actual angle) based on a set of PWM signals.
2. Generate a best-fit curve that allows for more accurate angle positioning by correcting these deviations.

The curve follows a linear formula based on datasheet information:
   - **PWM = 7.4 * desired_angle + 500**

For example, if we input `[0, 45, 90, 135, 180]` degrees, this should ideally correspond to `[500, 833, 1166, 1500, 1832]` PWM signals. However, due to hardware inconsistencies, actual angles may vary, so calibration is necessary.

## Process of Calibration

The following process will guide you through setting up and running the calibration:

1. Use cell 1 to retrive expected PWM for a known set of angles (`[0, 45, 90, 135, 180]`). PWM calculation are based on the ideal servo described in the datasheet.

2. Use the [arduino code](https://github.com/botzo-team/calibrate_servos/blob/main/calibarte_servos.ino) to allign the calibration tool in the desire angle and save the real PWM to reach desire angle.

3. Save PWM signals founded. For example, for my Front Right leg I had this:
   ```python
   real_pwm_SFR = np.array([564, 890, 1219, 1564, 1897])
   real_pwm_FFR = np.array([606, 930, 1265, 1606, 1930])
   real_pwm_TFR = np.array([555, 895, 1230, 1580, 1910])
   ```

4. Run Least Square regression and save a, b, c coefficients to fulfill the equation that best map desire angle in degree to PWM

5. Test new accuracy

### Cell 1: Setup and PWM Conversion

This cell initializes the setup and converts desired angles into PWM signals based on the known formula from the datasheet.

```python
# Libraries
import numpy as np

# Convert desired degrees to PWM based on datasheet formula
def deg2PWM(desire_deg_angles):
    output = []
    for angle in desire_deg_angles:
        pulse = round((7.4074 * angle) + 500, 0)
        output.append(pulse)
        print(f"// {angle} degrees => {pulse} PWM")
    return output

# Desired angles and corresponding PWM (based on formula)
desire_angles = [0, 45, 90, 135, 180, 270]
desired_PWM = deg2PWM(desire_angles)
print("\nDesired PWM values:", desired_PWM)

# Prompt to manually measure actual angles
print("\nPlease run the servo with the above PWM values and record the actual angles reached.")
```

### Arduino code
The arduino code, when uploaded, will move one servo to a desire angle.
Upload the code and open serial monitor. You will see the current PWM signal.
1. set it to 500
2. Attach the servo arm with the [calibration tool arm](https://github.com/botzo-team/STL_files/blob/main/servo_calibration_tools/arm_for_calibration_tool.stl) as close as possible to 0
3. Test some PWM to allign perfectly with 0
4. When reached the desire angle with maximum precision, write down the real PWM signal.
5. repeat process for the other known angles `[45, 90, 135, 180]`

### Cell 2

save results into array
such as:
```python
real_pwm_SFR = np.array([564, 890, 1219, 1564, 1897])
```

### Cell 3

Run regression and save the coefficent that best fit your servo ofsets. So when you will gove random angles, the accuracy of the servo is maximize!!!

```python
# Data: Degrees and corresponding real PWM values
degrees = np.array([0, 45, 90, 135, 180])  # x values (degree)

# Perform quadratic regression (degree 2 polynomial)
coefficients = np.polyfit(degrees, real_pwm, 2)

# Coefficients: a, b, c (for ax^2 + bx + c)
a, b, c = coefficients

# Output the coefficients
print(f"Quadratic equation coefficients: a = {a}, b = {b}, c = {c}")

# Optional: Plotting the fit
plt.scatter(degrees, real_pwm, color='blue', label='Data points')
x_vals = np.linspace(0, 180, 1000)
y_vals = a * x_vals**2 + b * x_vals + c
plt.plot(x_vals, y_vals, color='red', label=f'Quadratic fit: {a:.2f}x^2 + {b:.2f}x + {c:.2f}')
plt.xlabel('Degrees')
plt.ylabel('PWM')
plt.title('Quadratic Regression: Degrees vs Real PWM')
plt.legend()
plt.show()
```


## Notes
Ensure you measure the actual angles carefully, as they significantly impact the calibration accuracy.

Run this script separately for each servo on the robot dog, as each servo may have unique parameters.

By following these steps, you can effectively calibrate each servo for optimal performance in your robot dog application. This script provides an essential calibration layer, allowing your robot dog to achieve precise angles based on accurate PWM control.

Once the servo is calibrated and coefficents are saved do not remove the servo arm, or you will need to re-calibrate the servo.