#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <mavros_msgs/CommandBool.h> 
#include <mavros_msgs/SetMode.h>     
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>  
#include <mavros_msgs/PositionTarget.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCIn.h>


using namespace std;

// #define HIGHT	5		//初始飞行高度 m
// #define vel_z 0.3

bool marker_found = false,flag_move = false;
std::vector<int> current_target_id (1);  //定义一个值为零的向量
char mode;
double init_x_take_off =0, init_y_take_off =0, init_z_take_off =0;
float detec_x = 0, detec_y = 0, detec_z = 0;
double angle1 = 0,roll,pitch,yaw = 0,target_yaw = 0, diff_angle = 0, err_yaw = 0,err_yaw0 = 0, err_yaw_err = 0;
int value0,value1,value2,value3,value4,value5;
int HIGHT = 0,x_err = 0,y_err = 0 ;
double vel_z,cam_angle, Kp,MIN_ERROR;

mavros_msgs::State current_state;
mavros_msgs::PositionTarget setpoint; // 位置速度控制消息类
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// 订阅的无人机当前位置数据
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	local_pos = *msg;
}
void yaw_cb(sensor_msgs::Imu msg)
{
	tf::Quaternion quaternion(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
        );
    tf::Matrix3x3(quaternion).getRPY(roll,pitch,yaw);
    roll = roll*180/M_PI;
    pitch = pitch*180/M_PI;
    yaw = yaw*180/M_PI;

    //ROS_INFO("roll: %.0f ,pitch: %.0f  , yaw: %.0f",roll,pitch,yaw);  
}
void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	double vx,vy;
	if(!marker_found & mode == 'm')
	{
		vx = abs(msg->channels[1]  - value2) * 1.00 / abs(value0 - value2) * 1  -0.5; 
		vy = abs(msg->channels[3]  - value5) * 1.00/ abs(value5 - value3) * 1  -0.5; 
		if(abs(msg->channels[1] - value1) < 50) vx = 0;
		if(abs(msg->channels[3] - value5) < 50) vy = 0;
				ROS_INFO("vx=%f,vy:%f",vx,vy);

		double rotation_angle = ( angle1 ) * (M_PI / 180.0);
    	tf::Quaternion q;
		q.setRPY(0, 0, rotation_angle);

		tf::Vector3 point(vx,vy,0);
		tf::Matrix3x3 rotation_matrix(q);
		tf::Vector3 rotated_point = rotation_matrix * point;

		setpoint.velocity.x = rotated_point.x();
		setpoint.velocity.y = rotated_point.y();
	}
}
apriltag_ros::AprilTagDetection marker;
void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    int count = msg->detections.size();
	if(count!=0)
	{
		for(int i = 0; i<count; i++)
		{
			marker = msg->detections[i];			
			if(marker.id == current_target_id)
			{
				marker_found = true;
				detec_x = marker.pose.pose.pose.position.x;
        		detec_y = marker.pose.pose.pose.position.y;
        		detec_z = marker.pose.pose.pose.position.z; 
			}
		}
	//ROS_INFO("detec_x=%.2f,detec_y=%.2f,detec_z=%.2f",detec_x,detec_y,detec_z);
	}
	else
	{
		marker_found = false;
	}

}
void pos_xz(float x,float y)
{
	double vx,vy;
	double rotation_angle = ( angle1 ) * (M_PI / 180.0);
    tf::Quaternion q;
	q.setRPY(0, 0, rotation_angle);

	tf::Vector3 point(x,y,0);
	tf::Matrix3x3 rotation_matrix(q);
	tf::Vector3 rotated_point = rotation_matrix * point;

	ROS_INFO("vel_x:%f,vel_y:%f",rotated_point.x(),rotated_point.y());
	vx = Kp *  rotated_point.x() ;
	vy = Kp *  rotated_point.y() ;
	if(vx > 0.5) vx = 0.5;
	if(vx < -0.5) vx = -0.5;
	if(vy > 0.5) vy = 0.5;
	if(vy < -0.5) vy = -0.5;

	setpoint.velocity.x = vx;
	setpoint.velocity.y = vy;
}
void cam_xz(float xa,float ya)
{
	double xb,yb;
  	xb = xa * cos(cam_angle * (M_PI / 180.0) ) - ya * sin(cam_angle * (M_PI / 180.0));
   	yb =  - xa * sin(cam_angle * (M_PI / 180.0)) - ya * cos(cam_angle * (M_PI / 180.0));

	ROS_INFO("x_xz:%f,y_xz:%f",xb,yb);
    pos_xz(xb,yb);
}


int main(int argc, char *argv[])
{
	setlocale(LC_ALL, "");

	ros::init(argc,argv,"apriltag_land");
	ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);//订阅无人机状态话题
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);//订阅位置信息
	ros::Subscriber apriltag_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 10, apriltag_cb);//订阅识别信息
	ros::Subscriber yaw_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, yaw_cb);	//订阅无人机imu数据
	ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 10, rcin_cb);// 订阅摇杆杆量

    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);//控制话题,可以发布位置速度加速度同时控制
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20);

	nh.getParam("HIGHT", HIGHT);
	nh.getParam("vel_z", vel_z);
	nh.getParam("/cam/x_err", x_err);
	nh.getParam("/cam/y_err",y_err);
	nh.getParam("/cam/R",cam_angle);
	nh.getParam("Kp",Kp);
	nh.getParam("MIN_ERROR",MIN_ERROR);
	nh.getParam("/channle1/value0",value0);
	nh.getParam("/channle1/value1",value1);
	nh.getParam("/channle1/value2",value2);
	nh.getParam("/channle2/value3",value3);
	nh.getParam("/channle2/value4",value4);
	nh.getParam("/channle2/value5",value5);

	ROS_INFO("x_err:%d,y_err:%d",x_err,y_err);
	ROS_INFO("HIGHT:%d,vel_z:%f,R:%f",HIGHT,vel_z,cam_angle);

    while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
	}

    setpoint.header.stamp = ros::Time::now();
	setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	setpoint.type_mask =			//使用位置控制
	//mavros_msgs::PositionTarget::IGNORE_PX |
	//mavros_msgs::PositionTarget::IGNORE_PY |
	//mavros_msgs::PositionTarget::IGNORE_PZ |
	mavros_msgs::PositionTarget::IGNORE_VX |
	mavros_msgs::PositionTarget::IGNORE_VY |
	mavros_msgs::PositionTarget::IGNORE_VZ |
	mavros_msgs::PositionTarget::IGNORE_AFX |
	mavros_msgs::PositionTarget::IGNORE_AFY |
	mavros_msgs::PositionTarget::IGNORE_AFZ |
	mavros_msgs::PositionTarget::FORCE |
	mavros_msgs::PositionTarget::IGNORE_YAW ;
	//mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
	setpoint.position.x = 0;
	setpoint.position.y = 0;
	setpoint.position.z = 0;

    for(int i = 100; ros::ok() && i > 0; --i){
			setpoint.header.stamp = ros::Time::now();
		setpoint_pub.publish(setpoint);
			ros::spinOnce();
			rate.sleep();
	}

    mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	//设定无人机保护模式 POSTION
	mavros_msgs::SetMode offb_setPS_mode;
	offb_setPS_mode.request.custom_mode = "POSCTL";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	
	ros::Time last_request = ros::Time::now();

    mode = 't';
	int sametimes = 0;
	angle1 = yaw;
	target_yaw = yaw;
    while(ros::ok())
    {
		if (current_state.mode != "OFFBOARD" &&
		(ros::Time::now() - last_request > ros::Duration(5.0)))
		{
			if (set_mode_client.call(offb_setPS_mode) &&
				offb_setPS_mode.response.mode_sent)
			{
				ROS_INFO("POSTION PROTECTED");
			}
			last_request = ros::Time::now();
		}
		else
		{
			if (!current_state.armed &&
				(ros::Time::now() - last_request > ros::Duration(5.0)))
			{
				if (arming_client.call(arm_cmd) &&
					arm_cmd.response.success)
				{
					ROS_INFO("UAV armed");
					init_x_take_off = local_pos.pose.position.x;
					init_y_take_off = local_pos.pose.position.y;
					init_z_take_off = local_pos.pose.position.z;
				}
				last_request = ros::Time::now();
			}
			else
			{
				switch(mode)	
				{
					case 't':
					setpoint.position.x = init_x_take_off;
					setpoint.position.y = init_y_take_off;
					setpoint.position.z = init_z_take_off + HIGHT;
					if(local_pos.pose.position.z > init_z_take_off + HIGHT - 0.2)
					{	
						if (sametimes > 10)
              			{
         		           mode = 'm';
							last_request = ros::Time::now();
                    	}
            			else sametimes++;
                		}
						else sametimes = 0;
	 	            break;
					case 'm':
							setpoint.type_mask =			//使用速度控制
							mavros_msgs::PositionTarget::IGNORE_PX |
							mavros_msgs::PositionTarget::IGNORE_PY |
							mavros_msgs::PositionTarget::IGNORE_PZ |
							//mavros_msgs::PositionTarget::IGNORE_VX |
							//mavros_msgs::PositionTarget::IGNORE_VY |
							//mavros_msgs::PositionTarget::IGNORE_VZ |
							mavros_msgs::PositionTarget::IGNORE_AFX |
							mavros_msgs::PositionTarget::IGNORE_AFY |
							mavros_msgs::PositionTarget::IGNORE_AFZ |
							mavros_msgs::PositionTarget::FORCE |
							mavros_msgs::PositionTarget::IGNORE_YAW ;
							//mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

							if(marker_found)
								{
									cam_xz(detec_x,detec_y );
									setpoint.velocity.z = -vel_z;
									if(abs(detec_x) < MIN_ERROR && abs(detec_y) < MIN_ERROR)
									{
										setpoint.velocity.x = 0;
										setpoint.velocity.y = 0;
									}
								}
								else
								{
									setpoint.velocity.x = 0;
									setpoint.velocity.y = 0;
									setpoint.velocity.z = 0;
								}

							if(local_pos.pose.position.z  < init_z_take_off + 0.3)
							{
								mode = 'l';
								last_request = ros::Time::now();
							}
							break;
						case 'l':
							offb_set_mode.request.custom_mode = "AUTO.LAND";
							if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
							{
								if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
								{
									ROS_INFO("AUTO.LAND enabled");
								}
								last_request = ros::Time::now();
							}
							break;
                }

        	}
		}
		 //计算偏航角速度
		err_yaw = target_yaw - yaw;
		err_yaw_err =err_yaw - err_yaw0;
		err_yaw0 = err_yaw;
		diff_angle = 0.8 * err_yaw + 0.002 * err_yaw_err;
		setpoint.yaw_rate = diff_angle ;

		setpoint_pub.publish(setpoint);
		//ROS_INFO("marker_found=%d",marker_found);

		ros::spinOnce();
		rate.sleep();
	}

return 0;
}