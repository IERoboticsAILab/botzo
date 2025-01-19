# botzo_v2s_IK
Inverse Kinematics for botzo v2s

Inverse Kinematics is used in robots with legs or arms. It is a series of matrix moltiplications and trigonometry formulas (sin, cos,atan2 law of cosin, pitagora, etc...), in order too find the angles each motor in the leg need to have to reach a point in space (x,y,z). Our dog have 3 Degrees of freedom (3DoF), meaning 3 motors per leg.

_On the other side forward kinematics calculate where the end-effector/the foot end up based on given/known angles of each motor._

Both uses known constants such as lenght of each "bone". For Inverse Kinematic also the target point is a known variable, and it allow us to find the angle configuration we need to reach that point in space.

## 3 DoF IK Of 1 leg

<div align="center">
  <img src="https://github.com/botzo-team/our_images_and_videos/blob/main/IK_back_of_leg.png" alt="back_of_leg" width="500"/>
</div>

<div align="center">
  <img src="https://github.com/botzo-team/our_images_and_videos/blob/main/IK_side_of_leg.png" alt="side_of_leg" width="500"/>
</div>

### Solver

Note:

```
coxa = A = 3.1 cm
femur = E = 9.5 cm
real_femur = E' = 9.1 cm
tibia = F = 9.8 cm
θ(coxa) = ψ
θ(knee) = Φ
θ(shoulder) = θ
dist_focuspoint_servo_femurtibia = 2.8 cm
X, Y, Z = given target point
```

<div align="center">
  <img src="https://github.com/botzo-team/our_images_and_videos/blob/main/IK_trigonometrics_drawing.png" alt="trigonometrics_drawing" width="500"/>
</div>

1. Distance Calculation

```math
D = \sqrt{Z^2 + Y^2 - \text{A}^2}
```

2. G Calculation

```math
G = \sqrt{D^2 + X^2}
```

3. Knee Angle

```math
\text{Φ} = \arccos\left(\frac{G^2 - \text{E}^2 - \text{F}^2}{-2 \cdot \text{E} \cdot \text{F}}\right)
```

4. Shoulder Angle

```math
\theta_{\text{shoulder}} = \arctan2(X, D) + \arcsin\left(\frac{\text{F} \cdot \sin(\text{Φ})}{G}\right)
```

5. Coxa Angle

```math
\text{ψ} = \arctan2(Y, Z) + \arctan2(D, \text{A})
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

### Result

https://github.com/botzo-team/our_images_and_videos/blob/main/IK_3dof_first_result.mp4

![result1](https://github.com/botzo-team/our_images_and_videos/blob/main/IK_3dof_first_result-ezgif.com-video-to-gif-converter.gif)

In `coordinates_to_servos.ipynb` I have combined the use of our IK with the reansformation from degrees to PWM explained in [calibrate servos repo](https://github.com/botzo-team/calibrate_servos)

### Resources

[Spot-Micro](https://spotmicroai.readthedocs.io/en/latest/simulation/)
  - [Spot-Micro GitHub](https://gitlab.com/public-open-source/spotmicroai/simulation/-/tree/master/Basic%20simulation%20by%20user%20Florian%20Wilk/Kinematics?ref_type=heads)
  - [Spot-Micro IK solver video](https://www.youtube.com/watch?v=4rc8N1xuWvc)

[OpenQuadruped/spot_mini_mini](https://github.com/OpenQuadruped/spot_mini_mini)

## Rotation Matrices

[here](https://github.com/botzo-team/rotation_matrices_v1)

## IK full body

Use `IK_solcver4Lgs.ipynb`. It is a way to manualy retrive angles for each leg to make robot step and walking for the first time.

1. First the code loads known variables, such as the coefficents retrived from the calibration of each servo (find more [here](https://github.com/botzo-team/calibrate_servos)) and robot dimentions.

2. We define some usefull function for later (such as translate read to degrees)

3. We craete a IK solver for each leg. The only thing that changes is the adjustemts need for each leg servo angle. Because some servos has diffrent orientation therfor diffrent zero.

4. Then we manualy define points in the 3D space (x,y,z) inside an array. This points will then later be passed one by one to the IK solvers adn we will return PWM signals for the corrisponding angle that each servo need to have to reach that point.

5. We parse the result in a way the we can just copy and paste it in the `legsWalkFIRSTSTEP.ino` arduino code. The arduino code connect to each servo and move them coordinately. So we where able to manualy simulate a Walking Gate. For a more general and sophisticated gate look [here]()

![Leg Design](https://github.com/botzo-team/our_images_and_videos/blob/main/botzo.jpg)

<div style="display: flex; justify-content: center; gap: 10px;">
    <img src="https://github.com/botzo-team/our_images_and_videos/blob/main/firststeps.gif" alt="First Steps" width="300">
    <img src="https://github.com/botzo-team/our_images_and_videos/blob/main/walking.gif" alt="Walking" width="300">
</div>