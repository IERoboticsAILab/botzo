# botzo_v2s_IK
Inverse Kinematics for botzo v2s

## 3 DoF IK

<div align="center">
  <img src="https://github.com/botzo-team/our_images_and_videos/blob/main/IK_back_of_leg.png" alt="back_of_leg" width="500"/>
</div>

<div align="center">
  <img src="https://github.com/botzo-team/our_images_and_videos/blob/main/IK_side_of_leg.png" alt="side_of_leg" width="500"/>
</div>

### Solver

<div align="center">
  <img src="https://github.com/botzo-team/our_images_and_videos/blob/main/IK_trigonometrics_drawing.png" alt="trigonometrics_drawing" width="500"/>
</div>

1. Distance Calculation

```math
D = \sqrt{z^2 + y^2 - \text{coxa}^2}
```

2. G Calculation

```math
G = \sqrt{D^2 + x^2}
```

3. Knee Angle

```math
\theta_{\text{knee}} = \arccos\left(\frac{G^2 - \text{femur}^2 - \text{tibia}^2}{-2 \cdot \text{femur} \cdot \text{tibia}}\right)
```

4. Shoulder Angle

```math
\theta_{\text{shoulder}} = \arctan2(x, D) + \arcsin\left(\frac{\text{tibia} \cdot \sin(\theta_{\text{knee}})}{G}\right)
```

5. Coxa Angle

```math
\theta_{\text{coxa}} = \arctan2(y, z) + \arctan2(D, \text{coxa})
```

### Translator

Here we will translate the founded angles with the translator in the actual angles we will pass to the robot servos

<div align="center">
  <img src="https://github.com/botzo-team/our_images_and_videos/blob/main/IK_desire_angles.png" alt="actual_desire_angles" width="500"/>
</div>

1. Adjustment

```math
\text{adjustment} = \arccos\left(\frac{\text{real\_femur}^2 + \text{femur}^2 - \text{dist\_focuspoint\_servo\_femurtibia}^2}{2 \cdot \text{real\_femur} \cdot \text{femur}}\right)
```

2. Femur Angle

```math
\theta_{\text{femur}} = \frac{\pi}{2} - (\theta_{\text{shoulder}} + \text{adjustment})
```

3. Tibia Angle

```math
\theta_{\text{tibia}} = \pi - \theta_{\text{knee}} + \text{adjustment} + \theta_{\text{femur}}
```

