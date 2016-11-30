# README

## 1. Download and configure env
	1. `git clone git@192.168.1.100:~/dragon_rl.git`
		or `git clone git@192.168.1.100:/home/git/dragon_rl.git`
	2. `cd dragon_rl/dragon_ws/ && catkin_make`
	3. 如果编译失败， 再次编译， 或注释掉gps_agent_lib中生成库的指令， 首先生成Message， 之后再打开注释， 再次进行`catkin_make`
	4. `echo "source ~/dragon_rl/dragon_ws/devel/setup.bash" >> ~/.bashrc`
	5. `echo "export PYTHON_PATH=$PYTHON_PATH:~/dragon_rl/rl_impl/python" >> ~/.bashrc`
	6. roslaunch dragon_agent leg_agent_gazebo.launch 
	7. cd ~/rl_impl;  python python/gps/gps_main.py dragon_hopping_pi2_example

## 2. configure IDE??
