#include "util.h"
#include<opencv.hpp>

extern float g_plane_para[];



void build_plane_param(std::vector<float> &X, std::vector<float> &Y, std::vector<float> &Z, float* plane_params)
{
	cv::Mat src_mat(cv::Size(3, X.size()), CV_32FC1, cv::Scalar(0));
	for (int i = 0; i < X.size(); ++i)
	{
		src_mat.at<float>(i, 0) = X[i];//X coordinates 
		src_mat.at<float>(i, 1) = Y[i];//Y coordinates 
		src_mat.at<float>(i, 2) = Z[i];//Z coordinates 
	}




	float center_x = cv::mean(src_mat.col(0))[0];
	float center_y = cv::mean(src_mat.col(1))[0];
	float center_z = cv::mean(src_mat.col(2))[0];
	src_mat.rowRange(0, src_mat.rows).col(0) -= center_x;
	src_mat.rowRange(0, src_mat.rows).col(1) -= center_y;
	src_mat.rowRange(0, src_mat.rows).col(2) -= center_z;

	cv::Mat w, u, vt;

	cv::SVD::compute(src_mat, w, u, vt);
	plane_params[0] = vt.at<float>(2, 0);
	plane_params[1] = vt.at<float>(2, 1);
	plane_params[2] = vt.at<float>(2, 2);

	if (plane_params[0] * center_x + plane_params[1] * center_y + plane_params[2] * center_z > 0)
	{
		plane_params[0] = -plane_params[0];
		plane_params[1] = -plane_params[1];
		plane_params[2] = -plane_params[2];
	}

	plane_params[3] = -(plane_params[0] * center_x + plane_params[1] * center_y + plane_params[2] * center_z);

}
