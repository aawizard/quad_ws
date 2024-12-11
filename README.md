# Quadrotor Motion Optimization

## Overview
This project focuses on optimizing a quadrotor's motion for efficient battery swapping in a swarm system. A mathematical simulator was developed to test trajectories, ensuring consistency between simulation and real-world performance. Using a minimum-snap trajectory, the drone completes the task in 4 seconds compared to over 40 seconds with a brute-force PID controller, greatly improving efficiency.

### Main Video
<div style="text-align: center;">
  <video width="640" height="360" controls>
    <source src="https://github.com/user-attachments/assets/bb5eeca5-3b6b-43ec-a83a-5ed821dda174" type="video/webm">
    Your browser does not support the video tag.
  </video>
</div>

---

## Running the Code

### Simulation
To run the simulation:

```bash
ros2 launch quad_control launch_multiple.launch.xml
```

### Hardware
To run the code on hardware, follow these steps:

#### 1. Connecting with OptiTrack
1.1. Add a rigid body in the Motive app and run the OptiTrack script.

1.2. Connect the base station with Ethernet and configure the network:

```bash
sudo ifconfig eno0 192.168.1.2 netmask 255.255.255.0 up
```

Verify the connection is established, then run the `quad_listener` node:

```bash
ros2 run quad_listener quad_listener
```

#### 2. Connecting with the Drone
2.1. Connect the drone and the base station to the same network.

2.2. SSH into the Raspberry Pi:

```bash
ssh pi@<raspberry_pi_ip>
```

2.3. Run the drone control node to establish the connection with the drone:

```bash
ros2 run quad_control drone_control
```

2.4. On the Pi, execute the drone control script:

```bash
python drone.py
```

#### 3. Running the Controller
##### Joystick Control
```bash
ros2 run joy joy_node
ros2 run quad_control joystick
```

##### PID Controller
```bash
ros2 run quad_control controller_pid --ros-args -p des_x:=2.0 -p des_z:=1.0
```

##### Trajectory Controller
```bash
ros2 run quad_control controller_traj --ros-args -p des_x:=2.0 -p des_z:=1.0
