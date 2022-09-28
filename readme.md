# 配置步骤      

**这里推荐使用Ubuntu20.04** 
   
1. 下载源码     
   `git clone https://github.com/Luviewer/uavros_ws.git --recursive`
2. 安装`ROS`      
   参考：http://wiki.ros.org/     
   [这里请安装ros-noetic]  

3. 安装`uavros_ws`环境    
    步骤如下:  
    >`sudo apt-get install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros python3-wstool python3-catkin-tools protobuf-compiler`      

    >`sudo apt-get install libgeographic-dev ros-noetic-geographic-msgs  # Required for mavros.`     

    >`sudo apt-get install libgoogle-glog-dev`  

    >`cd ~/uavros_ws/src`

    >`catkin_init_workspace`    

    >`wstool init`

    >`cd ~/uavros_ws`

    >`catkin init  # If you haven't done this before. `
    
    >`catkin build`

    最后配置环境变量：

    >`echo "source ~/uavros_ws/devel/setup.bash" >> ~/.bashrc`    

    >`echo "export GAZEBO_MODEL_PATH=~/uavros_ws/src/uav_simulator/uav_gazebo/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc`  

    >`echo "export GAZEBO_RESOURCE_PATH=~/uavros_ws/src/uav_simulator/uav_gazebo/worlds:${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc`    

    >`source ~/.bashrc`

4. 安装ardupilot            
    参考: https://ardupilot.org/dev/index.html      
    1. 在任意位置下载源码:        
    `git clone https://github.com/Luviewer/ardupilot.git --recursive`     
    2. 安装ardupilot环境        
    `cd ardupilot`      
    `Tools/environment_install/install-prereqs-ubuntu.sh -y`        
    `. ~/.profile`

5. 安装VSCODE
   
6. 运行仿真   
   `roslaunch uav_gazebo spawn.launch`  
   `~/xx/ardupilot/ArduCopter/../Tools/autotest/sim_vehicle.py -f gazebo-iris --console` xx为`ardupilot`路径