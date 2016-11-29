1. git clone git@192.168.1.100:~/dragon_rl.git
2. cd dragon_rl/dragon_ws/ && catkin_make
3. 如果编译失败， 再次编译， 或注释掉gps_agent_lib中生成库的指令， 首先生成Message， 之后再打开注释， 再次进行`catkin_make`
4. echo "source ~/dragon_rl/dragon_ws/devel/setup.bash" >> ~/.bashrc
5. echo "export PYTHON_PATH=$PYTHON_PATH:~/dragon_rl/rl_impl/python" >> ~/.bashrc
