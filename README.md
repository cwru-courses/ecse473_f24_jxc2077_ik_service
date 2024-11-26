# ECSE473_f24_jxc2077_ik_service

> **Note**: This package is part of Laboratory #4 of ECSE/CSDS 373/473 at CWRU and provides functionality that supports ARIAC 2019-related tasks.

---

## **Features**
- Provides a ROS service (`pose_ik`) to calculate IK solutions for the UR10 robot.
- Outputs the number of solutions and corresponding joint angles for a given pose.
- Includes a client (`ik_client`) to test the service functionality.
- Fully integrated with the `ur_kinematics` package for IK computation.

---

## **Installation**

1. Clone the repository:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/<your_repo_url>/ik_service.git
   ```
2. Install dependencies:
   ```bash
   sudo apt install ros-noetic-ur-kinematics
   ```
3. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
4. Source the workspace:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

---

## **Usage**

### **Launch the Service Node**
Start the `ik_service` node using `roslaunch`:
```bash
roslaunch ik_service ik_service.launch
```

> **Expected Output**:
> ```
> [INFO] [<timestamp>] The pose_ik service has been successfully initialized.
> ```

### **Run the Client Node**
Run the `ik_client` node to send a pose request:
```bash
rosrun ik_service ik_client
```

> **Expected Output**:
> If successful:
> ```
> [INFO] [<timestamp>] Call to ik_service returned [2] solutions
> [INFO] [<timestamp>] Solution 0: [0.123, 1.234, -1.345, 0.567, 2.345, -0.123]
> [INFO] [<timestamp>] Solution 1: [-0.123, -1.234, 1.345, -0.567, -2.345, 0.123]
> ```
> If the service is unavailable:
> ```
> [ERROR] [<timestamp>] Failed to call service pose_ik
> ```

---

## **Node Descriptions**

### **Service Node**
- **Name**: `ik_service`
- **Service Name**: `pose_ik`
- **Inputs**: 
  - `geometry_msgs/Pose` specifying the desired end-effector position and orientation.
- **Outputs**:
  - Number of IK solutions (`num_sols`).
  - Array of joint angles for each solution.

### **Client Node**
- **Name**: `ik_client`
- **Functionality**:
  - Sends a test pose to the `pose_ik` service.
  - Prints the number of solutions and joint angles received from the service.

---

## **Testing**
To verify functionality:

1. **Check if the service node is running:**
   ```bash
   rosnode list
   ```
   You should see `/ik_service` in the output.

2. **Check service availability:**
   ```bash
   rosservice list
   ```
   You should see `/pose_ik` in the output.

3. **Manually call the service:**
   ```bash
   rosservice call /pose_ik "{part_pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
   ```
   Expected output:
   ```
   num_sols: 2
   joint_solutions:
   - joint_angles: [0.123, 1.234, -1.345, 0.567, 2.345, -0.123]
   - joint_angles: [-0.123, -1.234, 1.345, -0.567, -2.345, 0.123]
   ```

---

## **File Structure**

```
ik_service/
├── CMakeLists.txt
├── launch/
│   └── ik_service.launch
├── msg/
│   └── JointSolutions.msg
├── srv/
│   └── PoseIK.srv
├── src/
│   ├── ik_client.cpp
│   └── ik_service.cpp
├── include/
│   └── ik_service/
├── package.xml
└── README.md
```

---

## **Links**

- [ARIAC 2019 Documentation](http://ariac.osrfoundation.org/)
- [cwru_ariac_2019 Repository](https://github.com/cwru-ecse-373/cwru_ariac_2019)
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)

---

## **Acknowledgements**
Developed for **Modern Robot Programming ECSE/CSDS 373/473**, guided by Gregory S. Lee, Ph.D.
