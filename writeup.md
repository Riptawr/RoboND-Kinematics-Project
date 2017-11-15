## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/result.png
[image2]: ./misc_images/attempt_at_optimizing.png
[image3]: ./misc_images/kuka-dh.png
[image4]: ./misc_images/ik-top-down.png
[image5]: ./misc_images/ik-side.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Since we got the kr210.urdf.xarco file with a description of the robot model and the kr210 consists of 6, linearly connected, revolute joints,
deriving the DH parameters boils down to finding the offsets, link length and the angle of the joint, relative to the previous ones,
 as `a` and `alpha` do not change between orientations.
 This means we can express all poses with variations of `d` and `theta`.


![kr210-shematic][image3]

Below are the modified dh parameters derived from the drawing:

n   | alpha   | a     | d     | theta(n)
--- | ---     | ---   | ---   | ---
0   | 0       | 0     | -     | -
1   | -90     | .35   | .75   | q1
2   | 0       | 1.25  | 0     | q2
3   | -90     | -0.054| 0     | q3
4   |  90     | 0     | 1.5   | q4
5   | -90     | 0     | 0     | q5
6   | 0       | 0     | .303  | q6



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The following Python function was used, expecting the DH parameter table values for each joint(i) as arguments, and returns
the related transformation matrix, by substituting DH values into sympy symbols.
```python
def get_transformation_matrix(alpha, a, d, q, dh_params):
    """ Creates a modified transformation matrix"""

    return Matrix([
        [cos(q), -sin(q), 0, a],
        [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
        [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
        [0, 0, 0, 1]]).subs(dh_params)
```
Since the function is general, we can compose it multiple times, like composition of transformations in math.
 Such compositions are denoted **Ti_j** in the code - where i and j correlate to the starting and ending link of the DH table of said composition,
  e.g. **T0_3** for the composition **link 0 -> 1 -> 2 ->3.**

The resulting transformation matrices along each joint look as follows:

```python

T0_1
⎡cos(q₁)  -sin(q₁)  0   0  ⎤
⎢                          ⎥
⎢sin(q₁)  cos(q₁)   0   0  ⎥
⎢                          ⎥
⎢   0        0      1  0.75⎥
⎢                          ⎥
⎣   0        0      0   1  ⎦

T1_2
⎡cos(q₂ - 0.5⋅π)   -sin(q₂ - 0.5⋅π)  0  0.35⎤
⎢                                           ⎥
⎢       0                 0          1   0  ⎥
⎢                                           ⎥
⎢-sin(q₂ - 0.5⋅π)  -cos(q₂ - 0.5⋅π)  0   0  ⎥
⎢                                           ⎥
⎣       0                 0          0   1  ⎦

T2_3
⎡cos(q₃)  -sin(q₃)  0  1.25⎤
⎢                          ⎥
⎢sin(q₃)  cos(q₃)   0   0  ⎥
⎢                          ⎥
⎢   0        0      1   0  ⎥
⎢                          ⎥
⎣   0        0      0   1  ⎦

T3_4
⎡cos(q₄)   -sin(q₄)  0  -0.054⎤
⎢                             ⎥
⎢   0         0      1   1.5  ⎥
⎢                             ⎥
⎢-sin(q₄)  -cos(q₄)  0    0   ⎥
⎢                             ⎥
⎣   0         0      0    1   ⎦

T4_5
⎡cos(q₅)  -sin(q₅)  0   0⎤
⎢                        ⎥
⎢   0        0      -1  0⎥
⎢                        ⎥
⎢sin(q₅)  cos(q₅)   0   0⎥
⎢                        ⎥
⎣   0        0      0   1⎦

T5_6
⎡cos(q₆)   -sin(q₆)  0  0⎤
⎢                        ⎥
⎢   0         0      1  0⎥
⎢                        ⎥
⎢-sin(q₆)  -cos(q₆)  0  0⎥
⎢                        ⎥
⎣   0         0      0  1⎦

T6_E
⎡1  0  0    0  ⎤
⎢              ⎥
⎢0  1  0    0  ⎥
⎢              ⎥
⎢0  0  1  0.303⎥
⎢              ⎥
⎣0  0  0    1  ⎦

```

To get the homogeneous transform from base to gripper link (T0_1 -> T4_5) we compose the matrices in sequence.
```python
⎡(sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅cos(q₅) + sin(q₅)⋅cos(q₁)⋅cos(q₂ + q₃)  -(sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅sin(q₅) + cos(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)  -sin(q₁)⋅cos(q₄) + sin(q₄)⋅sin(q₂ + q₃)⋅cos(q₁)  (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅cos(q₁)⎤
⎢(sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅cos(q₅) + sin(q₁)⋅sin(q₅)⋅cos(q₂ + q₃)  (-sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) + sin(q₄)⋅cos(q₁))⋅sin(q₅) + sin(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)  sin(q₁)⋅sin(q₄)⋅sin(q₂ + q₃) + cos(q₁)⋅cos(q₄)   (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅sin(q₁)⎥
⎢                 -sin(q₅)⋅sin(q₂ + q₃) + cos(q₄)⋅cos(q₅)⋅cos(q₂ + q₃)                                      -sin(q₅)⋅cos(q₄)⋅cos(q₂ + q₃) - sin(q₂ + q₃)⋅cos(q₅)                                 sin(q₄)⋅cos(q₂ + q₃)                    -1.5⋅sin(q₂ + q₃) + 1.25⋅cos(q₂) - 0.054⋅cos(q₂ + q₃) + 0.75     ⎥
⎣                                           0                                                                                        0                                                                     0                                                           1                                  ⎦
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Conceptually, the approach consists of first moving the wrist center to a position,
 where the task is inside the manipulators' work space (the positioning step),
with the second step being to orient end-effector towards the object correctly, using the spherical wrist (the orientation step).

To calculate positioning, it is sufficient to consider only the first three joints, since they will move the wrist center:

![kr210-ik-top-down][image4]

![kr210-ik-side][image5]

The calculations to derive the angles can be obtained as follows:
```python
def get_joints_1_3(WC):
    a1, a2, a3 = 0.35, 1.25, -0.054
    d1, d4 = 0.75, 1.5

    # y and x position of the wrist center
    theta1 = atan2(WC[1], WC[0])

    # Using proper triangle terms for readability
    line_ab = a2  # empirically measured on the RViz model
    line_bc = sqrt(d4**2 + a3**2)
    line_ca = sqrt((sqrt(WC[0]**2 + WC[1]**2) - a1)**2 + (WC[2] - d1)**2)

    angle_a = acos((line_ca**2 + line_ab**2 - line_bc**2) / (2 * line_ca * line_ab))
    angle_b = acos((line_bc**2 + line_ab**2 - line_ca**2) / (2 * line_bc * line_ab))

    gamma = atan2(WC[2] - d1, sqrt(WC[0]**2 + WC[1]**2) - a1)
    beta = atan2(d4, -a3)

    theta2 = pi/2 - angle_a - gamma
    theta3 = -(angle_b - beta)  # accounting for sag in link 4 on z-axis

    return theta1, theta2, theta3
```


Next we solve the orientation step by using the angles above to derive the rotation matrix for the gripper:

```python
def get_rotation_matrix():
    r, p, y = symbols('r p y')
    R_x = Matrix([[1, 0, 0],
                  [0, cos(r), -sin(r)],
                  [0, sin(r), cos(r)]])

    R_y = Matrix([[cos(p), 0, sin(p)],
                  [0, 1, 0],
                  [-sin(p), 0, cos(p)]])

    R_z = Matrix([[cos(y), -sin(y), 0],
                  [sin(y), cos(y), 0],
                  [0, 0, 1]])

    R_E = R_z * R_y * R_x
    # Compensate for rotation discrepancy between DH parameters and Gazebo
    R_error_correction = R_z.subs(y, radians(180)) * R_y.subs(p, radians(-90))
    R_E = R_E * R_error_correction
    return R_E
```
 Note that there is no correlation between gazeboo parameters (from the planner) and DH parameters,
 therefore we apply error correction by rotating the gripper's coordinate frame by 180 degrees along Z and -90 degrees along Y.

The rotation matrix depends on the the angles T0_3, calculated beforehand,
 combined with the current roll, pitch and yaw of the gripper.

Since the planner already provides us with the poses for each step in the request,
 we can extract necessary **positions along the axis (px, py, pz)** and **orientation (roll, pitch, yaw)**
 and use them to derive the euler angles, by creating a transformation matrix from positions of the gripper and the rotation above:

            R_EE = R_E.subs({"r": roll, "p": pitch, "y": yaw})
            End_effector = Matrix([[px], [py], [pz]])
            WC = End_effector - 0.303 * R_EE[:, 2]


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Regular attempts are slow but precise, with lots of wrist re-orientations. Errors are only made if the gripper didn't have enough time to close
before the next movement (leading to cans spawning next to each other over the next runs)
![alt text][image1]

Attempts with optimizing the trajectory
(remembering WC orientation until the last 25% of the planed trajectory and only executing commands on joints 1-5)
While the speed is higher and movements seem more fluid, not orienting the wrist, based
on some conditions, depends on the task and cannot be generalized safely,
leading sometimes to flipping the can over:
![alt text][image2]

Further research could include deriving safe margins instead of calculating exact position, to make the movements more fluid
while not introducing potentially hazardous trajectories, but would probably need a re-thinking of the IK formulas used.