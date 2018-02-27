## Project: Kinematics Pick & Place
### This is the write up for the 6DOF Kuka KR210 serial manipulator pick and place project. The project involves forward and inverse kinematic analysis for calculating 6 joint angles that would bring the manipulator's end effector(gripper) to a given position and orientation.

---


**Steps to complete the project:**  


1. Setting up the ROS Workspace.
2. Downloading or clonning the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experimenting with the forward_kinematics environment and getting familiar with the robot.
4. Launching in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Performing Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Filling in the `IK_server.py` with the Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc2.png
[image2]: ./misc_images/KR210.png
[image3]: ./misc_images/Matrices.png
[image4]: ./misc_images/KR210_Theta123.png
[image5]: ./misc_images/R3_6_Matrix.png
[image6]: ./misc_images/KR210_Theta456.png

[image7]: ./misc_images/Arm1.png
[image8]: ./misc_images/Gazebo1.png
[image9]: ./misc_images/Gazebo2.png
[image10]: ./misc_images/Gazebo3.png
[image11]: ./misc_images/Gazebo4.png
[image12]: ./misc_images/Gazebo5.png
[image13]: ./misc_images/Rviz1.png
[image14]: ./misc_images/Rviz2.png
[image15]: ./misc_images/Rviz3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

Here is a snapshot of the KR210 manipulator carrying the blue cylinder towards the drop of location i.e. the bin in this case.

![alt text][image1]

### Kinematic Analysis
#### 1. Running the forward_kinematics demo and evaluating the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and deriving its DH parameters.

Drawing the arm on the paper greatly simplifies the process of deriving the modified DH parameter table. Below is a diagram of the manipulator in its initial configuration i.e all joint angles set to zero. The diagram also expresses the compensation for the difference of URDF values versus the way DH parameters are assigned. The DH parameter table has then been derived using the arm diagram.  

![alt text][image2]

#### 2. Using the DH parameter table derived earlier, individual transformation matrices about each joint and generalized homogeneous transform between base_link and gripper_link are being created using Sympy. Here's a how the matrices look from the jupyter notebook's output:

![alt text][image3]


#### 3. Decoupling the Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so deriving the equations to calculate all individual joint angles.

Using the pose(px,py,pz) and orientation(roll,pitch,yaw) of the end effector available from ROS, the wrist center(WC) can be found. After finding the WC, the equations for calculating theta1, theta2 and theta3 can be derived using geometrical analysis. The diagram below depicts the method: 

![alt text][image4]

The first three joint angles can then be substituted into R0_6 and R3_6 can be found using, R3_6 = R0_3.transpose() * R0_6, where R0_6 is the rotation matrix from base to the gripper. Here's how the R3_6 looks like from the jupyter notebook output:

![alt text][image5]

Equations for theta4, theta5 and theta6 can then be found using this R3_6 matrix. 

![alt text][image6]

### Project Implementation

#### 1. Using the `IK_debug.py` script, working of both the Forward and Inverse Kinematics calculations can be quickly verified against 3 different test cases. 

I was able to achieve an overall end effector error of 0.00002831 against all 3 test cases. More test cases can also be generated with Gazebo.  


#### 2. Filling in the `IK_server.py` file with python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. The code must guide the robot to successfully complete 8/10 pick and place cycles.  

The IK_server.py doesn't need the FK part but only the rotation matrix R0_3. I kept parameter symbol definitions, DH table and homogeneous matrices, code outside the for loop for increasing performance. However, everytime a Calculate IK request is made, these definitions etc are repeated which downgrades the manipulator's performance. Pickling the R_EE matrix which involves some matrix multiplications helped increase the performance a bit. Using numpy instead of sympy can further improves the performance.

The rest of the code follows naming the variables the same way as presented in the diagrams above. Below, I have added some screen shots from Gazebo and Rviz for a complete pick and place cycle. Running the kuka_arm ROS node with IK_server script and the safe_spawner.sh script would open up the Gazebo, Rviz and related processes.

![alt text][image7]

The expanded Rviz window:

![alt text][image13]

Different veiw perspectives are available in Gazebo and Rviz:

![alt text][image14]

Next are a few steps from pick and place cycle. Reaching target location:

![alt text][image8]

![alt text][image15]

Grasping target:

![alt text][image9]

Retrieving target:

![alt text][image10]

Reached drop off location:

![alt text][image11]

Dropping target:

![alt text][image12]

And [here](https://www.youtube.com/watch?v=OH4tmCHZ6Gk) is a video of a complete pick and place cycle.
