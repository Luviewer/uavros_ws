# 机型编译

## 编译IWP
1. 切换分支     
   `git checkout -b iwp_tilt origin/iwp_tilt`      
2. 编译运行arducopter     
   `cd ArduCopter`   
   `roslaunch uav_gazebo spawn.launch`         
   `../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map`        


## 编译四旋翼