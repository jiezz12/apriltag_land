# 无人机Apriltag码定位降落

## 工具

机载电脑 ubuntu18.04/20.04 

px4固件飞控

飞行平台

机载电脑需配置ROS+mavros+apriltag_ros

mavros安装参考:

配置流程分为源码安装和二进制（apt）安装

主要安装相机驱动和apriiltag推荐二进制安装，简单省时！



# 控制代码讲解

config:
//设置遥控器升降舵和方向舵杆量和若干参数
设置搜索模式点位x_move,y_move，即apriltag码附近点位,根据二维码实际大小和飞行高度进行速度PID修改


//流程：起飞上升到飞行高度然后进入搜索模式，靠升降舵和方向舵进行移动控制，如果识别的apriltag码则进入自动定位降落，手动无效。

流程：飞机先飞到搜索点位，然后进行二维码跟踪，如果长时间没有识别到apriltag码，则进入降落模式。


默认摄像头安装在飞机平面几何中心，相机坐标系y负方向指向机头



2024.11.21

增加模式切换，跟随模式和降落模式
