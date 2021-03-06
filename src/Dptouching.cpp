#include "Dptouching.h"
#include <math.h>
// #define max((a),(b)) ((a)>(b)?(a):(b))
// #define min((a),(b)) ((a)<(b)?(a):(b))
using namespace std;
DpTouching::DpTouching(ros::NodeHandle nh_):nh(nh_),nh_param("~")
{
	initialize();
}

void DpTouching::getBeginPos(float x, float y, float z)
{
	q = vector<Mat>(DP_N, Mat::zeros(12, 1, CV_32F));
	Mat q3 = Mat::zeros(12, 1, CV_32F);//临时变量
	q3.at<float>(0,0) = x;
	q3.at<float>(1,0) = y;
	q3.at<float>(2,0) = z;
	q.at(0) = q3;
}

void DpTouching::getTargetPos(float x, float y, float z)
{
	q_ugv = Mat::zeros(12, 1, CV_32F);
	q_ugv.at<float>(0,0) =x;
	q_ugv.at<float>(1,0) =y;
	q_ugv.at<float>(2,0) =z;

	//cout<<"x"<<x<<endl;
	//cout<<"y"<<y<<endl;
	//cout<<"z"<<z<<endl;

}

void DpTouching::runMethod(void)
{

	Mat A = Mat::eye(12, 12, CV_32F);
	for (int i = 0;i<12;i++)
	{
		for (int j = 0;j<12;j++)
		{
			if (i == (j-6))
			{
				A.at<float>(i,j) = dT;
			}
		}
	}

	Mat B = Mat::zeros(12, 6, CV_32F);//kinematics state space 
	for (int i = 0;i<12;i++)
	{
		for (int j = 0;j<6;j++)
		{
			if (i == j)
			{
				B.at<float>(i,j) =(dT*dT)/2;
			} 
			if (i == (j+6))
			{
				B.at<float>(i,j) = dT;
			}
		}
	}

	Mat P = Mat::zeros(12, 12, CV_32F);
	for (int i = 0;i<12;i++)
	{
		for (int j = 0;j<12;j++)
		{
			if (i == j && i<3)
			{
				P.at<float>(i,j) =0.5;
			} 
		}
	}

	Mat Q = Mat::eye(12, 12, CV_32F);
	Mat R = Mat::zeros(12, 6, CV_32F);
	Mat S = Mat::eye(6, 6, CV_32F);

	Mat gamma = 0.5*q_ugv.t()*P*q_ugv;
	Mat mu = -P*q_ugv;
	Mat eta = Mat::zeros(6, 1, CV_32F);

	Mat zeta1, nu1, W1;//临时变量，给类赋值，防止引用
	vector<Mat> zeta(DP_N, Mat::zeros(1, 1, CV_32F));
	gamma.copyTo(zeta1);
	zeta.at(DP_N-1) = zeta1;

	vector<Mat> nu(DP_N, Mat::zeros(12, 1, CV_32F));
	mu.copyTo(nu1);
	nu.at(DP_N-1) = nu1;

	vector<Mat> W(DP_N, Mat::zeros(12, 12, CV_32F));
	Q.copyTo(W1);
	W.at(DP_N-1) = W1;

	vector<Mat> H1(DP_N, Mat::zeros(12, 12, CV_32F));
	vector<Mat> H2(DP_N, Mat::zeros(12, 6, CV_32F));
	vector<Mat> H3(DP_N, Mat::zeros(6, 6, CV_32F));
	vector<Mat> h4(DP_N, Mat::zeros(12, 1, CV_32F));
	vector<Mat> h5(DP_N, Mat::zeros(6, 1, CV_32F));

	for (int i = DP_N-2;i>-1;i--)
	{
		//临时变量
		Mat H12;
		H12 = Q + A.t() * W.at(i+1) * A;
		H1.at(i) = H12;

		Mat H22;
		H22 = R + A.t() * W.at(i+1) * B;
		H2.at(i) = H22;

		Mat H32;
		H32 = S + B.t() * W.at(i+1) * B;
		H3.at(i) = H32;

		Mat h42;
		h42 = mu + A.t() * nu.at(i+1);
		h4.at(i) = h42;

		Mat h52;
		h52 = eta + B.t() * nu.at(i+1);
		h5.at(i) = h52;

		Mat zeta2;
		zeta2 = gamma + zeta.at(i+1) - 0.5 * h5.at(i).t() * H3.at(i).inv() * h5.at(i);
		zeta.at(i) = zeta2;

		Mat nu2;
		nu2 = h4.at(i) - H2.at(i) * H3.at(i).inv() * h5.at(i);
		nu.at(i) = nu2;

		Mat W2;
		W2 = H1.at(i) - 2 * H2.at(i) * H3.at(i).inv() * H2.at(i).t();
		W.at(i) = W2;
	}

	
	vector<Mat> u(DP_N, Mat::zeros(6, 1, CV_32F));


	for (int i = 0;i<DP_N-1;i++)
	{
		//临时变量
		Mat u4;
		u4 = -1 * H3.at(i).inv() * (H2.at(i).t() * q.at(i) + h5.at(i));
		u.at(i) = u4;

		Mat q4;
		q4 = A *q.at(i) + B * u.at(i);
		q.at(i+1) = q4;
	}


	//float throttle[DP_N];
	//float phi[DP_N];
	//float theta[DP_N];
	//float psi[DP_N];

	//float a_z[DP_N];

	for (int i = 0;i<DP_N;i++)
	{
		p_x[i] = q.at(i).at<float>(0,0);
		p_y[i] = q.at(i).at<float>(1,0);
		p_z[i] = q.at(i).at<float>(2,0);

		//a_z[i] = u.at(i).at<float>(2,0);

		//float uf5 = u.at(i).at<float>(0,0);
		//float uf6 = u.at(i).at<float>(1,0);
		//float uf7 = u.at(i).at<float>(2,0) + g;
		//throttle[i] = sqrt(uf5*uf5+uf6*uf6+uf7*uf7);
		//phi[i] = asin(-u.at(i).at<float>(1,0)/throttle[i]);
		//theta[i] = asin((u.at(i).at<float>(0,0)/cos(phi[i]))/throttle[i]);

	}
}

bool DpTouching::insideRec(float tx,float ty,float x1,float y1,float x2,float y2)
{
	if ((tx-x1)*(tx-x2)<0&&(ty-y1)*(ty-y2)<0)
	{
		return true;
	} 
	else
	{
		return false;
	}
}

void DpTouching::initialize()
{
	dT = 0.1;
	TG_server = nh.advertiseService("/TG/TG_service", &DpTouching::calculateTrajectoryCallback, this);
	quadrotorPosNED_sub = nh.subscribe("/dji_sdk/local_position", 10, &DpTouching::quadrotorPosNEDCallback, this);
	quadrotorPosGround_sub = nh.subscribe("ground_position",10, &DpTouching::quadrotorPosGroundCallback,this);
	hokuyoBody_sub = nh.subscribe("/hokuyo/pillar_data",10, &DpTouching::hokuyo_dataCallback, this);
	tf_client = nh.serviceClient<iarc_tf::Velocity>("ned_world_velocity_transform_srvice");
	guidance_distance_sub = nh.subscribe("/guidance/ultrasonic", 1, &DpTouching::guidanceObstacleCallback, this);
	guidance_emergency = false;
	hokuyo_emergency =false;
	if(!nh_param.getParam("xMax", xMax))xMax = 5.0;
	if(!nh_param.getParam("yMax", yMax))yMax = 5.0;
	if(!nh_param.getParam("avoidanceV", avoidanceV))avoidanceV = 0.6;
	if(!nh_param.getParam("cruiseVel", tarV))tarV = 0.5;
	if(!nh_param.getParam("Kr", Kr))Kr = 3.25;
	if(!nh_param.getParam("fattractive", fattractive))fattractive = 1.0;
	number_obstacle = 0;
}

void DpTouching::quadrotorPosNEDCallback(const dji_sdk::LocalPosition::ConstPtr &msg)
{
	quadrotorPosNED.x = msg->x;
	quadrotorPosNED.y = msg->y;
	quadrotorPosNED.z = msg->z;
}

void DpTouching::quadrotorPosGroundCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
	quadrotorPos.x = msg->point.x;
	quadrotorPos.y = msg->point.y;
}

void DpTouching::hokuyo_dataCallback(const obstacle_avoidance::Hokuyo::ConstPtr &msg)
{
	number_obstacle = msg->number;
	//ROS_INFO_THROTTLE(0.3,"DP: %3.1f,%3.1f",msg->ranges[0],msg->angles[0]);
	for(int i = 0; i < number_obstacle; i++)
	{
		obstacle_ranges[i] = msg->ranges[i];
		obstacle_angles[i] = msg->angles[i];
	}
}
void DpTouching::guidanceObstacleCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
	if ((msg->intensities[1] > 0.0) && (msg->ranges[1] < 2.5) && (msg->ranges[1] > 0.5))
	{
		guidancerang = msg->ranges[1];
		guidance_emergency = true;
		
	}
	else
	{
		guidance_emergency = false;
		guidancerang =999;
	}
}
bool DpTouching::calculateTrajectoryCallback(iarc_mission::TG::Request &req, iarc_mission::TG::Response &res)
{
	ros::spinOnce();
	float k = 0.8;
	float tarVz = k * (1.6 - quadrotorPosNED.z);
	switch(req.quadrotorState)
	{
		case FREE:
		{
			tarX = 0.0;
			tarY = 0.0;
			tarVx = 0.0;
			tarVy = 0.0;
			res.flightCtrlDstx = tarX;
			res.flightCtrlDsty = tarY;
			res.flightCtrlDstz = tarVz;
			res.flightFlag = 0x80;
			if(((number_obstacle > 0) && (obstacle_ranges[0] < 3.0))|| guidance_emergency)
			{
				repulsiveForce.setZero();
				calcRepulsiceForce();
				//ROS_INFO_THROTTLE(0.2,"doAvoidance: range=%4.2f,repulsiveForce=(%4.2f,%4.2f), attractiveForce=(%4.2f,%4.2f)",obstacle_ranges[0],repulsiveForce(0),repulsiveForce(1),attractiveForce(0),attractiveForce(1));
				//joinForce(0) = attractiveForce(0) + repulsiveForce(0);
				//joinForce(1) = attractiveForce(1) + repulsiveForce(1);
				joinForce = repulsiveForce;
				float theta_tar = atan2(joinForce(1), joinForce(0));
				tarVx = cos(theta_tar) * avoidanceV;	//NED
				tarVy = sin(theta_tar) * avoidanceV;
				res.flightCtrlDstx = tarVx;
				res.flightCtrlDsty = tarVy;
				res.flightCtrlDstz = tarVz;
				res.flightFlag = 0x40;
			}
			break;
		}
		case CRUISE:
		{
			static float lastTarVx = 0;//9.17-2
			static float lastTarVy = 0;//9.17-2
			tarZ = 1.6;
			ROS_INFO_THROTTLE(0.2, "CRUISE:posGround:%4.2f, %4.2f",quadrotorPos.x,quadrotorPos.y);

			float theta_center2quad = atan2((quadrotorPos.y-yMax/2),(quadrotorPos.x-xMax/2));//场地中心指向四旋翼的向量角度
			float dtheta = 5*dxy/(0.6/1.732*xMax);
// FOR 20*20 SIM SUCCESSFULLY			
			if((number_obstacle > 0) && (obstacle_ranges[0] < 3.0))
			{
				dtheta = 5*dxy/(0.6/1.732*xMax);
			}
			tarX = cos(theta_center2quad - dtheta) * 0.6/1.732*xMax + 0.5*xMax;
			tarY = sin(theta_center2quad - dtheta) * 0.6/1.732*yMax + 0.5*yMax;
			if(tarX > 0.8*xMax)
				tarX = 0.8*xMax;
			if(tarX < 0.2*xMax)
				tarX = 0.2*xMax;
			if(tarY > 0.8*yMax)
				tarY = 0.8*yMax;
			if(tarY < 0.2*yMax)
				tarY = 0.2*yMax;
// 			tarX = max(min(tarX,0.8*xMax),0.2*xMax);
// 			tarY = max(min(tarY,0.8*xMax),0.2*xMax);
/*			//-----9.18-----
			if((number_obstacle > 0) && (obstacle_ranges[0] < 3.0))
			{
				float verticalPointX = cos(theta_center2quad) * 0.6/1.414*xMax + 0.5*xMax;//9.18
				float verticalPointY = sin(theta_center2quad) * 0.6/1.414*yMax + 0.5*yMax;//9.18
// 				verticalPointX = max(min(verticalPointX,0.8*xMax),0.2*xMax);
// 				verticalPointY = max(min(verticalPointY,0.8*xMax),0.2*xMax);
				if(verticalPointX > 0.8*xMax)
					verticalPointX = 0.8*xMax;
				if(verticalPointX < 0.2*xMax)
					verticalPointX = 0.2*xMax;
				if(verticalPointY > 0.8*yMax)
					verticalPointY = 0.8*yMax;
				if(verticalPointY < 0.2*yMax)
					verticalPointY = 0.2*yMax;
				float parallelPointX = quadrotorPos.x + tarX - verticalPointX;
				float parallelPointY = quadrotorPos.y + tarY - verticalPointY;
				tarX = parallelPointX;
				tarY = parallelPointY;
			}
			//-----9.18 end-----
*/
			//-----9.18-----
/*			if((number_obstacle > 0) && (obstacle_ranges[0] < 3.0))
			{
				float verticalPointX = cos(theta_center2quad) * 0.6/1.414*xMax + 0.5*xMax;//9.18
				float verticalPointY = sin(theta_center2quad) * 0.6/1.414*yMax + 0.5*yMax;//9.18
				if(verticalPointX > 0.8*xMax)
					verticalPointX = 0.8*xMax;
				if(verticalPointX < 0.2*xMax)
					verticalPointX = 0.2*xMax;
				if(verticalPointY > 0.8*yMax)
					verticalPointY = 0.8*yMax;
				if(verticalPointY < 0.2*yMax)
					verticalPointY = 0.2*yMax;
				tarX = 10*(tarX - verticalPointX) + verticalPointX;
				tarY = 10*(tarY - verticalPointY) + verticalPointY;
			}*/
			float theta_quad2tar = atan2((tarY-quadrotorPos.y),(tarX-quadrotorPos.x));//四旋翼指向目标点的向量角度
			tarVx = cos(theta_quad2tar) * tarV;
			tarVy = sin(theta_quad2tar) * tarV;
			//---------9.17-2----------
			float a_V = 1;
			float dVx = tarVx - lastTarVx;
			float dVy = tarVy - lastTarVy;
			float length_dV = sqrt(dVx*dVx + dVy*dVy);
			float theta_dV = atan2(dVy,dVx);//dV向量角度
			tarVx = cos(theta_dV) * min(length_dV,a_V/50) + tarVx;
			tarVy = sin(theta_dV) * min(length_dV,a_V/50) + tarVy;
			
			lastTarVx = tarVx;//9.17-2
			lastTarVy = tarVy;//9.17-2
			//---------9.17-2 end----------
 
/*
			if (!insideRec(quadrotorPos.x,quadrotorPos.y,0.1*xMax,0.1*yMax,0.9*xMax,0.9*yMax))//在(0.1,0.1)(0.9,0.9)矩形外面//A
			{
				float theta_quad2center = atan2((yMax/2-quadrotorPos.y),(xMax/2-quadrotorPos.x));//四旋翼指向场地中心的向量角度
				tarVx = cos(theta_quad2center) * tarV;
				tarVy = sin(theta_quad2center) * tarV;
				tarX_ob = 0.5*xMax;
				tarY_ob = 0.5*yMax;
			}
			else
			{
				if(insideRec(quadrotorPos.x,quadrotorPos.y,0.3*xMax,0.3*yMax,0.7*xMax,0.7*yMax))//B
				{
					float theta_center2quad = atan2((quadrotorPos.y-yMax/2),(quadrotorPos.x-xMax/2));//场地中心指向四旋翼的向量角度
					tarVx = cos(theta_center2quad) * tarV;
					tarVy = sin(theta_center2quad) * tarV;
					tarX_ob = tarVx;
					tarY_ob = tarVy;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.7*xMax,0.3*yMax,0.9*xMax,0.9*yMax))//C
				{
					tarX = 0.8*xMax;
					tarY = quadrotorPos.y - dxy;
					float theta_quad2tar = atan2((tarY-quadrotorPos.y),(tarX-quadrotorPos.x));//四旋翼指向目标点的向量角度
					tarVx = cos(theta_quad2tar) * tarV;
					tarVy = sin(theta_quad2tar) * tarV;
					tarX_ob = 0.8*xMax;
					tarY_ob = 0.1*yMax;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.3*xMax,0.1*yMax,0.9*xMax,0.3*yMax))//D
				{
					tarX = quadrotorPos.x - dxy;
					tarY = 0.2*yMax;
					float theta_quad2tar = atan2((tarY-quadrotorPos.y),(tarX-quadrotorPos.x));//四旋翼指向目标点的向量角度
					tarVx = cos(theta_quad2tar) * tarV;
					tarVy = sin(theta_quad2tar) * tarV;
					tarX_ob = 0.1*xMax;
					tarY_ob = 0.2*yMax;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.1*xMax,0.1*yMax,0.3*xMax,0.7*yMax))//E
				{
					tarX = 0.2*xMax;
					tarY = quadrotorPos.y + dxy;
					float theta_quad2tar = atan2((tarY-quadrotorPos.y),(tarX-quadrotorPos.x));//四旋翼指向目标点的向量角度
					tarVx = cos(theta_quad2tar) * tarV;
					tarVy = sin(theta_quad2tar) * tarV;
					tarX_ob = 0.2*xMax;
					tarY_ob = 0.9*yMax;
				}
				else if(insideRec(quadrotorPos.x,quadrotorPos.y,0.1*xMax,0.7*yMax,0.7*xMax,0.9*yMax))//F
				{
					tarX = quadrotorPos.x + dxy;
					tarY = 0.8*yMax;
					float theta_quad2tar = atan2((tarY-quadrotorPos.y),(tarX-quadrotorPos.x));//四旋翼指向目标点的向量角度
					tarVx = cos(theta_quad2tar) * tarV;
					tarVy = sin(theta_quad2tar) * tarV;
					tarX_ob = 0.9*xMax;
					tarY_ob = 0.8*yMax;
				}
				else
				{
					tarVx = 0.1;
					tarVy = 0.2;
				}
			}*/
			
/* 			//--test-------
 			float theta_quad2center = atan2((0-quadrotorPos.y),(0-quadrotorPos.x));//四旋翼指向场地中心的向量角度
 			tarVx = cos(theta_quad2center) * tarV;
 			tarVy = sin(theta_quad2center) * tarV;
 			//---end--------
*/						
			iarc_tf::Velocity srv;
			srv.request.velocityFrame = GROUND;
			srv.request.velocityX = tarVx;
			srv.request.velocityY = tarVy;
			//ROS_ERROR("tarVx = %f",tarVx);
			if(tf_client.call(srv))
			{
				res.flightCtrlDstx = srv.response.velocityXRes;
				res.flightCtrlDsty = srv.response.velocityYRes;
				res.flightCtrlDstz = tarVz;
				res.flightFlag = 0x40;
			}
			else
				ROS_ERROR("Dptouching: tf call failed......");
			// has NED track vec, then if has obstacle:
			if(((number_obstacle > 0) && (obstacle_ranges[0] < 3.0))||guidance_emergency)
			{
				doAvoidance(Eigen::Vector2f(res.flightCtrlDstx, res.flightCtrlDsty));
				res.flightCtrlDstx = tarVx;
				res.flightCtrlDsty = tarVy;
				res.flightCtrlDstz = tarVz;
				res.flightFlag = 0x40;
			}
			
			break;
		}
			
		case TRACK:
		{
			//ROS_INFO("DpTouching: TRACK");
			tarX = req.irobotPosNEDx + 0.5 * cos(req.theta);
			tarY = req.irobotPosNEDy + 0.5 * sin(req.theta);
			tarZ = 1.6;

			getBeginPos(quadrotorPosNED.x,quadrotorPosNED.y,quadrotorPosNED.z);
			getTargetPos(tarX,tarY,tarZ);
			ROS_INFO_THROTTLE(0.2, "TRACK:quadPosNED=%4.2f,%4.2f,%4.2f,irobotPosNED=%4.2f,%4.2f,%4.2f",quadrotorPosNED.x,quadrotorPosNED.y,quadrotorPosNED.z,req.irobotPosNEDx,req.irobotPosNEDy,req.theta);
// 			runMethod();
// 			res.flightCtrlDstx = p_x[40];
// 			res.flightCtrlDsty = p_y[40];
// 			res.flightCtrlDstz = p_z[49];
			res.flightCtrlDstx = 0.6*(tarX-quadrotorPosNED.x);
			res.flightCtrlDsty = 0.6*(tarY-quadrotorPosNED.y);

/*			//-----9.17-------
			float maxP = 0.5;
			float finalP = 0.1;
			float middleDis = 0.5;
			float maxDis = 1;
			float conLength;
			float theta_quad2tar = atan2((tarY-quadrotorPosNED.y),(tarX-quadrotorPosNED.x));//四旋翼指向目标点的向量角度
			float dis_quad2tar = ((tarX-quadrotorPosNED.x)*(tarX-quadrotorPosNED.x)+(tarY-quadrotorPosNED.y)*(tarY-quadrotorPosNED.y));
			if(dis_quad2tar<middleDis)
			{
				conLength = (maxP-finalP)/middleDis * dis_quad2tar + finalP;
			}
			else
			{
				float k = (maxP-finalP)/(middleDis-maxDis);
				float b = maxP - k * middleDis;
				conLength = k*dis_quad2tar+b;
			}
			if(dis_quad2tar>maxDis)
			{
				conLength = finalP;
			}
			res.flightCtrlDstx = quadrotorPosNED.x + cos(theta_quad2tar)*conLength;
			res.flightCtrlDsty = quadrotorPosNED.y + sin(theta_quad2tar)*conLength;
			//------9.17 end ------
*/

			//res.flightCtrlDstz = 1.0*(tarZ-quadrotorPosNED.z)+quadrotorPosNED.z;
			res.flightCtrlDstz = tarVz;
			ROS_INFO_THROTTLE(0.2,"TRACK: dp_x=%4.2f,dp_y=%4.2f",res.flightCtrlDstx,res.flightCtrlDsty);
			res.flightFlag = 0x80;
			if(((number_obstacle > 0) && (obstacle_ranges[0] < 3.0)) || guidance_emergency)
			{
				doAvoidance(Eigen::Vector2f(tarX - quadrotorPosNED.x, tarY - quadrotorPosNED.y));
				res.flightCtrlDstx = tarVx;
				res.flightCtrlDsty = tarVy;
				res.flightCtrlDstz = tarVz;
				res.flightFlag = 0x40;
			}
			
			break;
		}
		case APPROACH:
		{
			//ROS_INFO("DpTouching: APPROACH");
			tarX = req.irobotPosNEDx + 2.0 * cos(req.theta);
			tarY = req.irobotPosNEDy + 2.0 * sin(req.theta);
			tarZ = -0.5;

			getBeginPos(quadrotorPosNED.x,quadrotorPosNED.y,quadrotorPosNED.z);
			getTargetPos(tarX,tarY,tarZ);
			ROS_INFO_THROTTLE(0.3, "APPROACH: quadPosNED=%4.2f,%4.2f,%4.2f,tarPosNED=%4.2f,%4.2f,%4.2f",quadrotorPosNED.x,quadrotorPosNED.y,quadrotorPosNED.z,tarX,tarY,tarZ);
// 			runMethod();
// 			res.flightCtrlDstx = p_x[40];
// 			res.flightCtrlDsty = p_y[40];
// 			res.flightCtrlDstz = p_z[49];
			res.flightCtrlDstx = 0.7*(tarX-quadrotorPosNED.x);//+quadrotorPosNED.x;
			res.flightCtrlDsty = 0.7*(tarY-quadrotorPosNED.y);//+quadrotorPosNED.y;
			//res.flightCtrlDstz = 1.2*(tarZ-quadrotorPosNED.z)+quadrotorPosNED.z;
			res.flightCtrlDstz = -0.6;
			res.flightFlag = 0x80; 
		
			if(number_obstacle > 0)
			{
				for(int i = 0;i != number_obstacle; i++)
				{
					if(obstacle_ranges[i] < 3.0)
					{
						iarc_tf::Velocity srv;
						srv.request.velocityFrame = GROUND;
						repulsiveVec(0) = obstacle_ranges[i];
						repulsiveVec(1) = obstacle_angles[i];
						srv.request.velocityX = -obstacle_ranges[i] * cos(-repulsiveVec(1));           
						srv.request.velocityY = -obstacle_ranges[i] * sin(-repulsiveVec(1));
						if(tf_client.call(srv))
						{
							//repulsiveForce(0) += srv.response.velocityXRes;
							//repulsiveForce(1) += srv.response.velocityYRes;
							obstcleNEDx = srv.response.velocityXRes + quadrotorPosNED.x;
							obstcleNEDy = srv.response.velocityYRes + quadrotorPosNED.y;
							float dis_obstacle_tar = sqrt((obstcleNEDx-tarX)*(obstcleNEDx-tarX)+(obstcleNEDy-tarY)*(obstcleNEDy-tarY));
							if(dis_obstacle_tar<2.5)
							{
								hokuyo_emergency = true;
								break;
							}
							else
								hokuyo_emergency = false;
						}
					}
				}
			}
			else
				hokuyo_emergency =false;
			if(guidance_emergency == true || hokuyo_emergency == true)
			{
				doAvoidance(Eigen::Vector2f(tarX - quadrotorPosNED.x, tarY - quadrotorPosNED.y));
				res.flightCtrlDstx = tarVx;
				res.flightCtrlDsty = tarVy;
				res.flightCtrlDstz = quadrotorPosNED.z;
				res.flightFlag = 0x50;
			}
			break;
		}
	}
	return true;
}


void DpTouching::doAvoidance(Eigen::Vector2f attVec)
{
	setAttVec(attVec);		//set attractiveVec
	calcAttractiveForce();	//calc attractive force
	repulsiveForce.setZero();
	if(number_obstacle > 0 || guidance_emergency)
	{
		calcRepulsiceForce();
	}
	ROS_INFO_THROTTLE(0.2,"doAvoidance: range=%4.2f,ang=%4.2f,repulsiveForce=(%4.2f,%4.2f), attractiveForce=(%4.2f,%4.2f)",obstacle_ranges[0],obstacle_angles[0],repulsiveForce(0),repulsiveForce(1),attractiveForce(0),attractiveForce(1));
	//joinForce(0) = attractiveForce(0) + repulsiveForce(0);
	//joinForce(1) = attractiveForce(1) + repulsiveForce(1);
	joinForce = attractiveForce + repulsiveForce;
	float theta_tar = atan2(joinForce(1), joinForce(0));
	tarVx = cos(theta_tar) * avoidanceV;	//NED
	tarVy = sin(theta_tar) * avoidanceV;
}


void DpTouching::setAttVec(Eigen::Vector2f attVec)
{
	attractiveVec(0) = attVec(0);
	attractiveVec(1) = attVec(1);
}

void DpTouching::setRepVec(Eigen::Vector2f repVec)
{
	repulsiveVec(0) = repVec(0);
	repulsiveVec(1) = repVec(1);
}


void DpTouching::getForce(Eigen::Vector2f &attForce, Eigen::Vector2f &repForce)
{
	attForce(0) = attractiveForce(0);
	attForce(1) = attractiveForce(1);
	repForce(0) = repulsiveForce(0);
	repForce(1) = repulsiveForce(1);
}

void DpTouching::calcAttractiveForce()
{
	float theta_goal = atan2(attractiveVec(1),attractiveVec(0));
	attractiveForce(0) = fattractive * cos(theta_goal);
	attractiveForce(1) = fattractive * sin(theta_goal);
	//ROS_ERROR("%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f",attractiveVec(0),attractiveVec(1),theta_goal,fattractive,attractiveForce(0),attractiveForce(1));
}

void DpTouching::calcRepulsiceForce()
{
	repulsiveForce.setZero();
	if(number_obstacle > 0)
	{
		for(int i = 0;i != number_obstacle;i++)
		{
			if(obstacle_ranges[i] < 3.0)
			{
				iarc_tf::Velocity srv;
				srv.request.velocityFrame = GROUND;
				repulsiveVec(0) = obstacle_ranges[i];
				repulsiveVec(1) = obstacle_angles[i];
				srv.request.velocityX = Kr/(repulsiveVec(0)*repulsiveVec(0)) * cos(-repulsiveVec(1));           
				srv.request.velocityY = Kr/(repulsiveVec(0)*repulsiveVec(0)) * sin(-repulsiveVec(1));
				if(tf_client.call(srv))
				{
					repulsiveForce(0) += srv.response.velocityXRes;
					repulsiveForce(1) += srv.response.velocityYRes;
				}
				//ROS_INFO_THROTTLE(0.3,"repulsiveVec=%3.1f,%3.1f,%3.1f,%3.1f,%3.1f",repulsiveVec(0),repulsiveVec(1),Kr/(repulsiveVec(0)*repulsiveVec(0)),repulsiveForce(0),repulsiveForce(1));
				
			}
		}
	}
	if(guidance_emergency)
	{
		iarc_tf::Velocity srv;
		srv.request.velocityFrame = GROUND;
		srv.request.velocityX = -6.5/guidancerang*guidancerang;          
		srv.request.velocityY = 0;
		if(tf_client.call(srv))
		{
			repulsiveForce(0) += srv.response.velocityXRes;
			repulsiveForce(1) += srv.response.velocityYRes;
		}
	}
}


