# 机型编译

## 编译IWP
1. 切换分支     
   `git checkout -b iwp_tilt origin/iwp_tilt`      
2. 编译运行arducopter     
   `cd ArduCopter`   
   `roslaunch uav_gazebo spawn.launch`         
   `../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map`        


## 仿真四旋翼
1. UWB四旋翼  
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --add-param-file=Tools/autotest/default_params/gazebo-tsduav_quad.parm -w