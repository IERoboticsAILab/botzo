# Servo Calibration

!!! note

    This guide can be quite improved.

Servos work by using PWM (Pulse Width Modulation) signals, where different pulse  
widths correspond to specific angles. Typically, this ranges between 500 µs and  
2000 µs. However, due to manufacturing tolerances—especially in more affordable  
servos—there can be significant variation in how each servo responds to the same  
PWM signal.

To improve accuracy, each servo can be calibrated to determine its specific PWM  
response curve. This only requires some simple math and a bit of setup.

## Result

| Before Calibration                                                        | After Calibration                                                       |
| ------------------------------------------------------------------------- | ----------------------------------------------------------------------- |
| ![Before Calibration](../assets/gifs/before_calibration.gif){width="50%"} | ![After Calibration](../assets/gifs/after_calibration.gif){width="50%"} |

With calibration, servo motion becomes noticeably more accurate and consistent.

## Repo Content

1. `Calibration_Procedure.ipynb`: A Google Colab notebook that fits a least-squares  
   regression to convert angles to PWM values specific to each servo.

2. `calibarte_servos.ino`: Arduino code to help identify real PWM values using the  
   [Calibration Tool](https://github.com/IERoboticsAILab/botzo/tree/main/CAD_files/designs/servo_calibration_tools).

3. `save_coefficients/`: Folder containing `.csv` and `.xlsx` files used to store  
   calibration data for each leg and servo.

## Background and Motivation

Servos are controlled by PWM signals to reach a given angle. However, each servo  
has unique mechanical and electrical characteristics that lead to deviations between  
the **desired** angle and the **actual** angle achieved.

This calibration process helps:

1. Measure how each servo responds to standard angles.
2. Fit a custom curve (linear or quadratic) for more accurate angle-to-PWM mapping.

The theoretical formula from datasheets is often:

- **PWM = 7.4 × desired_angle + 500**

So for example:

- `[0, 45, 90, 135, 180]` degrees → `[500, 833, 1166, 1500, 1832]` PWM values

However, these are idealized values—calibration helps find the real-world equivalent.

## Process of Calibration

1. **Compute ideal PWM values**  
   Use Cell 1 of the notebook to calculate PWM values for known angles using the  
   datasheet formula.

2. **Align servo using Arduino code**  
   Upload the [Arduino sketch](https://github.com/IERoboticsAILab/botzo/blob/main/control/inverse_kinematics/servo_calibration/calibarte_servos.ino)  
   and use the serial monitor to manually tune PWM values until the servo aligns with  
   target angles (0°, 45°, 90°, 135°, 180°).

3. **Record the actual PWM values**  
   Example values from the Front Right leg:

   ```python
   real_pwm_SFR = np.array([564, 890, 1219, 1564, 1897])
   real_pwm_FFR = np.array([606, 930, 1265, 1606, 1930])
   real_pwm_TFR = np.array([555, 895, 1230, 1580, 1910])
   ```

4. **Fit a regression curve**  
   Use least-squares regression to generate coefficients for the curve that maps  
   desired degrees to PWM for each servo.

5. **Test and verify accuracy**  
   Use the new function to convert angles into PWM and verify improved motion accuracy.

---

## Cell 1 – Setup and PWM Conversion

```python
# Libraries
import numpy as np

# Convert desired angles (degrees) to PWM using datasheet formula
def deg2PWM(desire_deg_angles):
    output = []
    for angle in desire_deg_angles:
        pulse = round((7.4074 * angle) + 500, 0)
        output.append(pulse)
        print(f"// {angle} degrees => {pulse} PWM")
    return output

# Example usage
desire_angles = [0, 45, 90, 135, 180, 270]
desired_PWM = deg2PWM(desire_angles)
print("\nDesired PWM values:", desired_PWM)

print("\nPlease run these PWM signals and record the actual angles reached.")
```

---

## Arduino Procedure

1. Upload the Arduino sketch.
2. Open the serial monitor and set the PWM to 500.
3. Attach the servo arm using the [calibration tools](https://github.com/IERoboticsAILab/botzo/tree/main/CAD_files/designs/servo_calibration_tools) aligned as closely as possible to 0°.
4. Adjust the PWM value until the arm aligns perfectly with 0°.
5. Record the PWM, and repeat for 45°, 90°, 135°, and 180°.

---

## Cell 2 – Save Real PWM Values

```python
real_pwm_SFR = np.array([564, 890, 1219, 1564, 1897])
```

---

## Cell 3 – Run Regression

```python
import matplotlib.pyplot as plt

# Input: degrees and corresponding real PWM values
degrees = np.array([0, 45, 90, 135, 180])
real_pwm = real_pwm_SFR  # Replace with your servo data

# Fit a quadratic regression curve
coefficients = np.polyfit(degrees, real_pwm, 2)
a, b, c = coefficients

print(f"Quadratic coefficients: a = {a}, b = {b}, c = {c}")

# Plotting
plt.scatter(degrees, real_pwm, color='blue', label='Measured data')
x_vals = np.linspace(0, 180, 1000)
y_vals = a * x_vals**2 + b * x_vals + c
plt.plot(x_vals, y_vals, color='red', label=f'Fit: {a:.2f}x² + {b:.2f}x + {c:.2f}')
plt.xlabel('Degrees')
plt.ylabel('PWM')
plt.title('Servo Calibration Curve')
plt.legend()
plt.grid(True)
plt.show()
```

---

## Notes

- Calibrate **each servo** individually, as they may have different characteristics.
- Keep the servo horn attached after calibration—removing it will require redoing the process.
- Store your coefficients securely for consistent motion across reboots or deployments.

By following this process, you’ll significantly improve servo positioning accuracy,  
making your robot’s motion more reliable and repeatable.
