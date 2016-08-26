#include "Dptouching.h"
#include <math.h>

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
	////判断飞行器状态
	//switch (state)
	//{
	//case CRUISE:
	//	q_ugv.at<float>(2,0) = cruise_height;
	//	break;
	//case TRACK:
	//	q_ugv.at<float>(2,0) = track_height;
	//	break;
	//case APPROACH:
	//	q_ugv.at<float>(2,0) = approach_height;
	//	break;
	//default:
	//	break;
	//}

	//cout<<q_ugv<<endl;

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

bool insideRec(float tx,float ty,float x1,float y1,float x2,float y2)
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