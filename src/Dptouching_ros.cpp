#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include "Dptouching.h"
#include <goal_detected/Pose3D.h>
#include <dji_sdk/LocalPosition.h>
#include "iarc_mission/TG.h"
using namespace std;


#define xMax 20.0						//���ط�Χ#8.27
#define yMax 20.0						//���ط�Χ#8.27
//xMin �� yMin Ϊ 0

//����
/*
Dp_pos quadrotorPos = Dp_pos(1.5,1.5,2),//������λ��
	irobotPos = Dp_pos(2,3,0);//С��λ��
float irobotDir = 0;//С������
//���
Dp_pos outputPos;//���λ��
*/
//DPstate quadrotorState = APPROACH;//������״̬
dji_sdk::LocalPosition quadrotorPos;
goal_detected::Pose3D irobotPos;
geometry_msgs::Point32 outputPos;
float tarX, tarY, tarZ;
float tarVx, tarVy, tarV = 0.5;//tarV:Ѳ���ٶȴ�С
DpTouching DpTouch;
void quadrotorPosCallback(const dji_sdk::LocalPosition::ConstPtr &msg)
{
	quadrotorPos.x = msg->x;
	quadrotorPos.y = msg->y;
	quadrotorPos.z = msg->z;
}
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
			tarZ = 1.6;
			if (insideRec(quadrotorPos.x,quadrotorPos.y,0.1*xMax,0.1*yMax,0.9*xMax,0.9*yMax))//��(0.1,0.1)(0.9,0.9)��������
			{
				if(insideRec(quadrotorPos.x,quadrotorPos.y,0.15*xMax,0.15*yMax,0.85*xMax,0.85*yMax))
				{
					double theta_center2quad = atan2((quadrotorPos.y-yMax/2),(quadrotorPos.x-xMax/2));//��������ָ��������������Ƕ�
					tarVx = cos(theta_center2quad)*tarV;
					tarVy = sin(theta_center2quad)*tarV;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.1*xMax,0.1*yMax,0.85*xMax,0.15*yMax))
				{
					tarVx = tarV;
					tarVy = 0;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.9*xMax,0.1*yMax,0.85*xMax,0.85*yMax))
				{
					tarVx = 0;
					tarVy = tarV;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.9*xMax,0.85*yMax,0.15*xMax,0.9*yMax))
				{
					tarVx = -tarV;
					tarVy = 0;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.15*xMax,0.15*yMax,0.1*xMax,0.9*yMax))
				{
					tarVx = 0;
					tarVy = -tarV;
				}
				else
				{
					tarVx = 0;
					tarVy = 0;
				}

			}
			else//��(0.1,0.1)(0.9,0.9)��������
			{
				double theta_quad2center = atan2((yMax/2-quadrotorPos.y),(xMax/2-quadrotorPos.x));//������ָ�򳡵����ĵ������Ƕ�
				tarVx = cos(theta_quad2center)*tarV;
				tarVy = sin(theta_quad2center)*tarV;
			}
			res.flightCtrlDstx = tarVx;
			res.flightCtrlDsty = tarVy;
			res.flightCtrlDstz = tarZ;
			break;
		case TRACK:
			ROS_INFO("DpTouching: TRACK");
			tarX = req.irobotPosNEDx + 0.5 * cos(req.theta);
			tarY = req.irobotPosNEDy + 0.5 * sin(req.theta);
			tarZ = 1.6;

			DpTouch.getBeginPos(quadrotorPos.x,quadrotorPos.y,quadrotorPos.z);
			DpTouch.getTargetPos(tarX,tarY,tarZ);
			ROS_INFO("%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f",quadrotorPos.x,quadrotorPos.y,quadrotorPos.z,tarX,tarY,tarZ);
			DpTouch.runMethod();
			res.flightCtrlDstx = DpTouch.p_x[40];
			res.flightCtrlDsty = DpTouch.p_y[40];
			res.flightCtrlDstz = DpTouch.p_z[49];
			break;
		case APPROACH:
			ROS_INFO("DpTouching: APPROACH");
			tarX = req.irobotPosNEDx + 2.3 * cos(req.theta);
			tarY = req.irobotPosNEDy + 2.3 * sin(req.theta);
			tarZ = -0.5;

			DpTouch.getBeginPos(quadrotorPos.x,quadrotorPos.y,quadrotorPos.z);
			DpTouch.getTargetPos(tarX,tarY,tarZ);
			ROS_INFO("%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f",quadrotorPos.x,quadrotorPos.y,quadrotorPos.z,tarX,tarY,tarZ);
			DpTouch.runMethod();
			res.flightCtrlDstx = DpTouch.p_x[40];
			res.flightCtrlDsty = DpTouch.p_y[40];
			res.flightCtrlDstz = DpTouch.p_z[49];
			break;
	}
	return true;

}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Dptouching_node");
	ros::NodeHandle nh;
	ros::Subscriber quadrotorPos_sub = nh.subscribe("/dji_sdk/local_position", 10, quadrotorPosCallback);
	//ros::Subscriber irobotPos_sub = nh.subscribe("/goal_detected/goal_pose", 10, irobotPosCallback);
	//ros::Publisher outputPos_pub = nh.advertise<geometry_msgs::Point32>("/TG/flight_ctrl_dst",10);
	ros::ServiceServer TG_server = nh.advertiseService("/TG/TG_service", calculateTrajectoryCallback);
	ros::spin();
	return 0;
}
