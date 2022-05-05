# CS3891-Project

## Members: Ziyang Li, Ericka Liu, Xinyu Niu

## Modified Files:

	1. surgical_robotics_challenge/scripts/surgical_robotics_challenge/launch_crtk_interface.py
  
	2. surgical_robotics_challenge/scripts/surgical_robotics_challenge/psm_arm.py
  
	3. surgical_robotics_challenge/scripts/surgical_robotics_challenge/examples/crtk_ros_based_control.py

## Instruction on Code Executing
	
```bash
roscore
```
  
In a new terminal tab, run: 

```bash
./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,14,15 -p 120 -t 1 --override_max_comm_freq 120
```
  
In a new terminal tab, cd to the surgical_robotics_challenge/scripts/surgical_robotics_challenge folder, run: 

```bash
python launch_crtk_interface.py
```
  
In a new terminal tab, cd to the examples folder, run: 

```bash
python crtk_ros_based_control.py
```

## Video Demo
<p align="center">
<img src="Demo.mov">
</p>
