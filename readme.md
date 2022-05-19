# 配置步骤      

**这里推荐使用Ubuntu20.04** 

1. 安装`ROS`      
   参考：http://wiki.ros.org/     
   [这里请安装ros-noetic]  

2. 安装`uavros_ws`环境    
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

3. 