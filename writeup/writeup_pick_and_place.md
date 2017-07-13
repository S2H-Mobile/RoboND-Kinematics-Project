# Project: Kinematics Pick & Place

**Steps to complete the project:**  

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[dh-transform-matrix]: ./images/dh-transform-matrix.png
[drawing_dh_frames]: ./images/drawing_dh_frames_kr210.jpg
[dh-parameter-table]: ./images/dh-parameter-table.png
[urdf-parameter-table]: ./images/urdf-parameter-table.png
[drawing-sss-triangle]: ./images/drawing_sss_triangle.jpg
[wrist-center-plane]: ./images/wrist-center-plane.jpg
[theta3-offset]: ./images/theta3-offset.jpg


# [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

## Kinematic Analysis
In this project we study the kinematics of the KUKA KR210 robotic arm. This antropomorphic manipulator comprises of six rotational degrees of freedom.

The task is to solve the forward and inverse kinematics problems.

- The forward kinematics problem is to find the end-effector pose given the values for the generalized coordinates.
- The inverse kinematics problem is finding the configuration of generalized coordinates that results in the requested end-effector pose.

The KR210 manipulator is designed as a spherical wrist. The last three degrees of freedom have rotational axes that intersect at a common point, the wrist center. The advantage of such a design is that it kinematically decouples the position and orientation of the end effector. This simplifies the inverse kinematics problem, since instead of solving twelve nonlinear equations simultaneously (one equation for each term in the first three rows of the overall homogeneous transform matrix), it is now possible to independently solve two simpler problems:

1. find the Cartesian coordinates of the wrist center (inverse position),
2. find the composition of rotations to orient the end-effector (inverse orientation).

### 1. Run the `forward_kinematics` demo and evaluate the `kr210.urdf.xacro` file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To analyse the KR210 manipulator I first read the joint properties off of the  `kr210.urdf.xacro` file.

![URDF parameter table for KR210.][urdf-parameter-table]

Then I made a drawing of the robot arm and assigned reference frame ``O_i`` to joint ``i`` according to the Denavit-Hartenberg prescription. I followed the assignment algorithm from the lectures and described the algorithm in detail in section 2.1 of the [supporting material](kinematics.pdf). This is the resulting schematic drawing of the robot arm:

![Drawing of DH frames for KR210.][drawing_dh_frames]

Subsequent frames are connected by a homogeneous transformation ``T``. The transformation from frame ``i-1`` to frame ``i`` is characterized by four parameters,

1. twist angle ``alpha_(i-1)``,
2. link length ``a_(i-1)``,
3. link offset ``d_i``,
4. joint angle ``theta_i``.

The properties of these four parameters are described in more detail in section 2.1 of the [supporting material](kinematics.pdf).

Then I wrote down the corresponding DH parameter values in tabular form, see table 2. Each row in the table corresponds to a homogeneous transformation ``T`` from frame ``i-1`` to frame ``i``.

![DH parameter table for KR210.][dh-parameter-table]

I used the ``forward_kinematics`` demo to determine the default position of the six joints as well as the direction of the joint variables ``theta1`` to ``theta6``. The results are presented as part of the drawings under point 3.

### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between `base_link` and `gripper_link` using only end-effector (gripper) pose.

I used Jupyter notebook for doing matrix algebra and TeX for presenting the matrices.

I summarized the mathematics of homogeneous transformation in section 2.1 of the [supporting material](kinematics.pdf). Resulting from this, the general form of the homogeneous transformation matrix as a function of the Denavit-Hardenberg parameters is given by

![homogeneous transform][dh-transform-matrix]

I used this as a template to calculate the individual transformation matrices as well as the overall transformation in the [kinematics notebook](../kinematics_test.ipynb).

Then I multiplied them to calculate the overall transformation matrix and used ``trigsimp`` function of the ``sympy`` package to simplify it. This makes use of the fact that the individual transormation matrices are sparse when evaluated using the the given parameters for the KR210.

I also took into consideration the different frame assignments used in the URDF description and the DH description. The resulting back-transformation I describe section 3.2 of the [supporting material](kinematics.pdf). See also the [kinematics notebook](../kinematics_test.ipynb), where the generalized homogeneous transform between `base_link` and `gripper_link` is denoted as ``T_0_G_0``.

### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The last three joints 4, 5 and 6 of the KR210 are revolute. They constitute a spherical wrist since their axes intersect at a single point, the wrist center at joint 5. So the inverse kinematics problem is decoupled into

- the position problem for the wrist center and
- the orientation problem for the end effector.

I describe the solution for a general spherical wrist in section 2.3 of the [supporting material](kinematics.pdf). 

I used [this video lecture](https://www.youtube.com/watch?v=llUBbpWVPQE), which has been recommended by the teaching assistants on Slack.

#### Part 1: Inverse Position
##### Wrist Center
First I use vector algebra to calculate the wrist center coordinates in terms of the base frame ``O_0``. The end effector position and orientation are given by the request as a vector ``p`` and a roation matrix ``R`` in frame ``O_0``.

The column ``i`` of a rotation matrix is the base vector ``i`` of the rotated frame in coordinates of the original frame, and therefore of unit norm. In this case the first column vector points in the direction of ``p - w``, and the distance between the wrist center ``w`` and the gripper joint ``p`` is ``d4 = 0.303``. So the wrist center is determined by

``
w = p - d4 * R[:0]
``.

Then I follow the video to define a new *wrist center frame*, which will help to determine ``theta2`` and ``theta3`` later on. The origin of this frame is given by  ``joint_2`` (or equivalently Denavit-Hartenberg frame ``O_2``). The orientation of the wrist center frame is defined by the xy-plane, which intersects the wrist center ``joint_5`` and the ``Z_0`` axis, see the drawing below. 

![wrist center plane][wrist-center-plane]

Then I calculate the coordinates ``x_c`` and ``y_c`` of the wrist center in the new frame, see the formulas on the drawing above.

``x_c = norm([w[0], w[1]]) - a1``

``y_c = w[2] - d1``

The point ``(x_c, y_c)`` will be soon very helpful in defining a triangle for finding ``theta2`` and ``theta3``.

##### ``theta1``
The first angle is fairly straightforward since the tangens of ``theta1`` is given by the ratio of the ``Y_0`` and ``X_0`` components of either the wrist center ``w`` or equivalently the end-effector position ``p`` in frame ``O_0``, see drawing above. So the joint variable is given by

```
theta1 = atan2(p[1], p[0])
```

There is a second solution ``theta1 + pi`` that can be choosen when the arm is in a configuration where the wrist center ``X_0`` is below zero, i.e. the arm is "reaching backwards". Looking at the forward kinematics demo in Rviz, this solution seems to be inside the working range of the KR210 arm design. Nevertheless in the simulation this configuration seems to be outside the planning range (the white box in gazebo), so I choose the first solution.   

##### ``theta2``
The lines ``l23``, ``l25`` and ``l35`` constitute the three sides of a sss triangle, see the figure below. The triangle determines the the angles ``phi2`` and ``phi3`` by the cosine law. These angles can be used to calculate the joint variables ``theta2`` and ``theta3``, respectively. The equation yielding the second joint variable is given by

```
theta2 = pi/2 - phi2 - alpha
```

where ``alpha`` is the angle between x-axis and ``l25``.

![sss triangle][drawing-sss-triangle]

##### ``theta3``
Most of the work to determine ``theta3`` is done at this point. Only the offset ``a3`` between joint 3 and the wrist center needs to be taken into account. The trigonometry depicted below leads to the equation for joint variable 3,

```
theta3 = pi/2 - phi3 - delta
```.

The offset is accommodated for by the angle ``delta = atan2(a3,d4)``.

![offset for theta3][theta3-offset]

#### Part 2: Inverse Orientation

The task is to find the orientation of the end-effector, the joint variables 4, 5 and 6. The key observation is that those are the Euler angles of the rotation ``R3_6``.

The matrix ``R3_6`` is determined by decomposing the overall rotation matrix ``R0_6 = R0_3 * R3_6`` and comparing it to the requested orientation,

``R0_3 * R3_6 = R``.

Multiplying both sides by the inverse rotation ``R3_0 = inverse(R0_3)`` from the left gives ``R3_6 = R3_0 * R``.

The matrix has been calculated before. Now it is evaluated at the joint values from the first part of the problem to give the equation  

``R3_6 = R3_0_eval * R``.

I denote the matrix element ``R3_6[i-1][j-1]`` as ``rij``. The remaining joint variables are then given by the Euler angles for ``R3_6``,

``theta4 = atan2( r32, r33)``,

``theta6 = atan2( r21, r11)``,

``theta5 = atan2(-r31, sqrt(r11**2 + r21**2))``.

I describe the geometry of Euler angles in section 1 of the [supporting material](kinematics.pdf). 

## Project Implementation

### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

#### Implementation
My implementation consists of two files. 

- The python module ``IK_server.py`` contains the code to be run as a ROS server in the simulation. The relevant functions in the module are named ``forward_kinematics()`` and ``inverse_kinematics(p, R)``.

- The function ``forward_kinematics()`` calculates the matrix ``R3_0`` that rotates frame 3 back to frame 0.  The matrix ``R3_0`` is needed for the orientation part of the inverse problem. The forward kinematics function makes use of the DH parameter table (the constant named ``s`` in the module) and the homogeneous transform matrices ``T`` defined in the [kinematics notebook](../kinematics_test.ipynb).

- The function ``inverse_kinematics(p, R)`` implements the spherical wrist solution strategy described above to calculate a joint angle configuration for the requested end-effector pose, which is given as a position vector ``p`` and an orientation matrix ``R``. The function calculates the wrist center, uses the trigonometry described above to solve the position problem which yields the first three joint angles. Then it solves the orientation problem by determining the rotation matrix ``R3_6``. This is done by evaluating the rotation ``R3_0`` at the calculated configuration for the first three joint variables and then multiplying it with the rotation ``R`` given by the request. Finally the Euler angles for ``R3_6`` are calculated.

- The [kinematics notebook](../kinematics_test.ipynb) contains all the homogeneous transform matrices ``T`` for the forward kinematics frame transformations as well as a duplicate of the inverse kinematics code for testing purposes.

#### Results
I tested successfully the forward and inverse kinematics for a series of cases in the [kinematics notebook](../kinematics_test.ipynb). This can be reproduced in the last cell in the notebook. A test case is entered as an array ``q`` of six angles. Both the forward and inverse kinematics are run for this configuration, and the result is compared to the input.

There is a mismatch for joint 4 and 6 due to the rotational singularity that occurs when ``theta5 = 0``. Then the requested roll for a pose has infinitely many solutions ``theta4 + theta6 = requested roll``, since a degree of freedom is lost. This results in an offset by a multiple of ``pi/2`` for both ``theta4`` and ``theta6``, however they add up to a valid solution.

In the simulation, the arm did pick up most of the samples successfully but also pushed some of the samples before grasping them.

Unfortunately the slowness and lack of stability of the simulation environment was a problem for me, e.g. recording a screencast video of the pick and place operation was not feasible.

#### Improvements
##### Precision
For inverse kinematics, the main task is to reduce the numerical errors since they add up for trajectories making the manipulator less and less accurate. The numerical errors are caused by evaluating trigonometric functions and lengths (norms) of vectors as power series.

- Avoid chaining numerical errors by simplifying the trigonometry as much as possible. As an example, try not to use the sine of an angle that has been calculated with the cosine law.

- Check the validity of rotation matrices, for example check the determinant and that the rows/columns add up to unity. Enforce the ortonormality properties as necessary, for example by rescaling the components until orthonormality is fullfilled.

##### Speed up the simulation

Regarding project code:

- The execution time can be reduced by predefining all constant entities that only depend on the DH table, instead of calculating them when ``IK_server.py`` is run. This applies mainly to the fixed transformation ``R_3_0``, but also the constant offsets and distances that are used in the function ``inverse_kinematics()``.

Regarding the environment: 

- Install the Ubuntu/ROS environment natively.

- Invest in more performant hardware.