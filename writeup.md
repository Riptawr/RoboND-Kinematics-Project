## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/result.png
[image2]: ./misc_images/attempt_at_optimizing.png
[image3]: ./misc_images/kr210-dh.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Since we got the kr210.urdf.xarco file with a description of the robot model and the kr210 consists of mostly spherical joints,
deriving the DH parameters boils down to finding the offsets, link length and the angle of the joint, relative to the previous ones.

![kr210-shematic][image3]

Below are the modified dh parameters:

n   | alpha   | a     | d     | theta(n)
--- | ---     | ---   | ---   | ---
0   | 0       | 0     | -     | -
1   | -pi / 2.| .35   | .75   | q1
2   | 0       | 1.25  | 0     | q2 - pi / 2.
3   | -pi / 2 | -0.054| 0     | q3
4   | pi / 2. | 0     | 1.5   | q4
5   | -pi / 2 | 0     | 0     | q5
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


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Conceptually, the approach consists of first moving the wrist center (the last three joints) to a position, where the task is inside the manipulators' work space,
with the second step being to orient end-effector towards the object correctly, using the wrist.

Since the planner already provides us with the poses for each step,
 we can extract necessary **positions along the axis (px, py, pz)** and **orientation (roll, pitch, yaw)** from the request
 and apply them to the rotation matrix, using the transformation matrices of the first three joints (T0_3) only (to save cpu cycles)

 Note that there is no correlation between gazeboo parameters (from the planner) and DH parameters,
 so a correction function was included into the rotation matrix function, to account for that, as hinted at in the lecture material:

```python
     R_error_correction = R_z.subs(y, radians(180)) * R_y.subs(p, radians(-90))
```

Since the IK problem can be divided into two functions on paper, similar functions can be created in code using sympy.
I leave out the paper calculations for brevity, since code comments go into performance optimization details, as well.

Given the wrist center position matrix, joint angles for first three links can be calculates as a right triangle:
```python
def get_joints_1_3(WC):
    # Avoiding re-eval of costly sympy variables inside loop
    # those parameters are constants anyway
    a1, a2, a3 = 0.35, 1.25, -0.054
    d1, d4 = 0.75, 1.5

    # y and x position of the wrist center
    theta1 = atan2(WC[1], WC[0])

    # Using proper triangle terms, the references are harder to read without the drawing, see writeup.
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

The wrist center is calculated in a similar fashion
```python
def get_joints_4_6(rot_matrix):
    theta5 = atan2(sqrt(rot_matrix[0, 2] ** 2 + rot_matrix[2, 2] ** 2), rot_matrix[1, 2])
    if sin(theta5) < 0:
        theta4 = atan2(-rot_matrix[2, 2], rot_matrix[0, 2])
        theta6 = atan2(rot_matrix[1, 1], -rot_matrix[1, 0])
    else:
        theta4 = atan2(rot_matrix[2, 2], -rot_matrix[0, 2])
        theta6 = atan2(-rot_matrix[1, 1], rot_matrix[1, 0])

    return theta4, theta5, theta6
```

After the transformations are done outside the loop, we calculate the spherical wrist's rotation matrix:
```python
            R_EE = R_E.subs({"r": roll, "p": pitch, "y": yaw})
            End_effector = Matrix([[px], [py], [pz]])
            WC = End_effector - 0.303 * R_EE[:, 2]

            # Calculate joint angles using Geometric IK method
            theta1, theta2, theta3 = get_joints_1_3(WC)

            # We only substitute and eval here
            R0_3 = T0_3[0:3, :3]  # Reusing already calculated first 3 joints outside the loop, instead of costly multipy + inverse
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv("LU") * R_EE
```

With the (already corrected) rotation matrix, retrieving the euler angles is simple via the function above:
```python
theta4, theta5, theta6 = get_joints_4_6(rot_matrix=R3_6)
```



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Regular attempts are slow but precise, with lots of wrist-center re-orientations. Errors are only made if the gripper didn't have enough time to close
before the next movement (leading to cans spawning next to each other over the next runs)
![alt text][image1]

Attempts with optimizing the trajectory
(remembering WC orientation until the last 25% of the planed trajectory)
While the speed is higher and movements seem more fluid, not orienting the wrist center, based
on some conditions, depends on the task and cannot be generalized safely,
leading sometimes to flipping the can over:
![alt text][image2]

