#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include "Dptouching.h"
#include <goal_detected/Pose3D.h>
#include <dji_sdk/LocalPosition.h>
#include "iarc_mission/TG.h"
#include <tf/transform_listener.h>
#include <sys/socket.h>

using namespace std;
//输入
/*
Dp_pos quadrotorPos = Dp_pos(1.5,1.5,2),//四旋翼位置
	irobotPos = Dp_pos(2,3,0);//小车位置
float irobotDir = 0;//小车方向
//输出
Dp_pos outputPos;//输出位置
*/
//DPstate quadrotorState = APPROACH;//四旋翼状态
dji_sdk::LocalPosition quadrotorPos;
goal_detected::Pose3D irobotPos;
geometry_msgs::Point32 outputPos;
float tarX, tarY, tarZ;
float dxy = 2.0;
bool transformState;
DpTouching DpTouch;
/*
void quadrotorPosCallback(const dji_sdk::LocalPosition::ConstPtr &msg)
{
	quadrotorPos.x = msg->x;
	quadrotorPos.y = msg->y;
	quadrotorPos.z = msg->z;
}
*/
/*
void irobotPosCallback(const goal_detected::Pose3DConstPtr &msg)
{
	irobotPos.x = msg->x;
	irobotPos.y = msg->y;
	irobotPos.z = msg->z;
	irobotPos.theta = msg->theta;
}
*/
bool calculateTrajectoryCallback(iarc_mission::TG::Request &req, iarc_mission::TG::Response &res)
{
	
	switch(req.quadrotorState)
	{
		case CRUISE:	//TODO: here is not NED frame!!!
			ROS_INFO("DpTouching: CRUISE");
			tarZ = 1.8;	
			if (insideRec(quadrotorPos.x,quadrotorPos.y,0,0,18,2))//outside
			{
				tarX = quadrotorPos.x + dxy;
				tarY = 1;
			}
			else if (insideRec(quadrotorPos.x,quadrotorPos.y,18,0,20,18))
			{
				tarX = 19;
				tarY = quadrotorPos.y + dxy;
			}
			else if (insideRec(quadrotorPos.x,quadrotorPos.y,2,18,20,20))
			{
				tarX = quadrotorPos.x - dxy;
				tarY = 19;
			}
			else if (insideRec(quadrotorPos.x,quadrotorPos.y,0,2,2,20))
			{
				tarX = 1;
				tarY = quadrotorPos.y - dxy;
			}
			else if (insideRec(quadrotorPos.x,quadrotorPos.y,2,2,10,10))//里边
			{
				tarX = quadrotorPos.x;
				tarY = quadrotorPos.y - dxy;
			}
			else if (insideRec(quadrotorPos.x,quadrotorPos.y,10,2,18,10))
			{
				tarX = quadrotorPos.x + dxy;
				tarY = quadrotorPos.y;
			}
			else if (insideRec(quadrotorPos.x,quadrotorPos.y,10,10,18,18))
			{
				tarX = quadrotorPos.x;
				tarY = quadrotorPos.y + dxy;
			}
			else if (insideRec(quadrotorPos.x,quadrotorPos.y,2,10,10,18))
			{
				tarX = quadrotorPos.x - dxy;
				tarY = quadrotorPos.y;
			}
			else//出边线
			{
				tarX = 10;
				tarY = 10;
			}
			break;
		case TRACK:
			ROS_INFO("DpTouching: TRACK");
			tarX = req.irobotPosNEDx + 0.5 * cos(req.theta);
			tarY = req.irobotPosNEDy + 0.5 * sin(req.theta);
			tarZ = 1.6;
			break;
		case APPROACH:
			ROS_INFO("DpTouching: APPROACH");
			tarX = req.irobotPosNEDx + 2.3 * cos(req.theta);
			tarY = req.irobotPosNEDy + 2.3 * sin(req.theta);
			tarZ = -0.5;
			break;
	}
	DpTouch.getBeginPos(quadrotorPos.x,quadrotorPos.y,quadrotorPos.z);
	DpTouch.getTargetPos(tarX,tarY,tarZ);
	ROS_INFO("%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f",quadrotorPos.x,quadrotorPos.y,quadrotorPos.z,tarX,tarY,tarZ);
	DpTouch.runMethod();
	res.flightCtrlDstx = DpTouch.p_x[40];
	res.flightCtrlDsty = DpTouch.p_y[40];
	res.flightCtrlDstz = DpTouch.p_z[49];
	return true;
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Dptouching_node");
	ros::NodeHandle nh;
	//ros::Subscriber quadrotorPos_sub = nh.subscribe("/dji_sdk/local_position", 10, quadrotorPosCallback);
	//ros::Subscriber irobotPos_sub = nh.subscribe("/goal_detected/goal_pose", 10, irobotPosCallback);
	//ros::Publisher outputPos_pub = nh.advertise<geometry_msgs::Point32>("/TG/flight_ctrl_dst",10);
	ros::ServiceServer TG_server = nh.advertiseService("/TG/TG_service", calculateTrajectoryCallback);
	
	tf::TransformListener listener;
	ros::Rate rate(10.0);
	
	
	while(ros::ok()){
	        //ros::spinOnce();
		tf::StampedTransform transform;
		try{
			listener.waitForTransform("/body","/ground",ros::Time(0),ros::Duration(10.0));    
			listener.lookupTransform("/body","/ground",ros::Time(0),transform);
			quadrotorPos.x = transform.getOrigin().x();
			quadrotorPos.y = transform.getOrigin().y();
			
			cout << "Current position:(" << quadrotorPos.x << "," << quadrotorPos.y << ")" << endl;
		}
		catch(tf::TransformException &ex){
			ROS_ERROR("%s",ex.what());
		}
		rate.sleep();
	}
	
	return 0;
}
