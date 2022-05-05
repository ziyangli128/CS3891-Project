# CS3891-Project

Members: Ziyang Li, Ericka Liu, Xinyu Niu

Modified Files:

	1. surgical_robotics_challenge/scripts/surgical_robotics_challenge/launch_crtk_interface.py
  
	2. surgical_robotics_challenge/scripts/surgical_robotics_challenge/psm_arm.py
  
	3. surgical_robotics_challenge/scripts/surgical_robotics_challenge/examples/crtk_ros_based_control.py

Instruction on Code Executing
	
	1. roscore
  
	2. In a new terminal tab, run: ./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,14,15 -p 120 -t 1 --override_max_comm_freq 120
  
	3. In a new terminal tab, cd to the surgical_robotics_challenge/scripts/surgical_robotics_challenge folder, run: python launch_crtk_interface.py
  
	4. In a new terminal tab, cd to the examples folder, run: python crtk_ros_based_control.py
