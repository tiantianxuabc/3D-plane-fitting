#include <iostream>

#include <opencv.hpp>

#include<fstream>

#include "pca.h"
#include "util.h"



#if _DEBUG
#pragma comment(lib, "opencv_world430d.lib")
#else
#pragma comment(lib, "opencv_world430.lib")
#endif

typedef  PCA_Plane plane_info;

float g_plane_para[4]{0.f};
std::ofstream fout("analysis.csv", std::ios::app);


int main()
{
	std::vector<float> z_mean_std;
	std::string path = "D:/point_06_05_2020_14_47_54.txt";

	int H = 576, W = 640;
	cv::Mat point_data(H, W, CV_32FC3);
	std::ifstream fin(path, std::ios::in);
	char line[1024] = { 0 };
	float x = 0;
	float y = 0;
	float z = 0;

	int index = 0;
	while (fin.getline(line, sizeof(line)) && index < W*H)
	{
		std::stringstream word(line);
		word >> x;
		word >> y;
		word >> z;
		point_data.at<cv::Vec3f>(index++) = cv::Vec3f(x, y, z);

	}

	point_data *= 1000.f;
	
	std::cout << point_data.rows << " " << point_data.cols << " " << point_data.channels() << std::endl;;



	
	for (int i = 0; i < point_data.rows;i++)
	{
		for (int j = 0; j < point_data.cols; j++)
		{
			float point = point_data.at<cv::Vec3f>(i, j)[2];
			if (point > 1.f)
			{
				z_mean_std.push_back((point ));
			}
		}
	}


	


	cv::Scalar z_mean, z_std;
	cv::meanStdDev(z_mean_std, z_mean, z_std);
	std::cout << path << std::endl;
	std::cout << "point size: " << z_mean_std.size() << std::endl;
	std::cout << "z_mean:\t" << z_mean[0] << "\tz_std:\t" << z_std[0] << std::endl;
	fout << "\n" << "" << ",z_mean = " << z_mean[0] << "," << "z_std = " << z_std[0] << "\n";


	std::vector<cv::Mat> pd_n;
	cv::split(point_data, pd_n);
	cv::Mat depth_color;
	pd_n[2].convertTo(depth_color, CV_8UC1, 50.0 / 5000);
	applyColorMap(depth_color, depth_color, cv::COLORMAP_JET);
// 	cv::Mat pd_img;
// 	normalize(pd_n[2], pd_img, 0, 255, cv::NORM_MINMAX);
	cv::imshow("depth_color", depth_color);
	cv::imwrite("depth_color_image.jpg", depth_color);
	cv::waitKey(0);



	std::vector<float> c_X;
	std::vector<float> c_Y;
	std::vector<float> c_Z;
	plane_info plane;

	for (int i = 0; i < W*H; i++)
	{
		cv::Vec3f p = point_data.at<cv::Vec3f>(i);
		
		if (p[2] > 1.f)
		{
			c_X.push_back(p[0]);
			c_Y.push_back(p[1]);
			c_Z.push_back(p[2]);
			plane.push(p[0], p[1], p[2]);
		}
		
	}

	

	float param[4] = { 0.f };
	build_plane_param(c_X, c_Y, c_Z, param);	
	plane.compute();

	cv::Vec3f normal(param[0], param[1], param[2]);
	std::vector<float> dist;
	for (int i = 0; i < point_data.size().area(); i++ )
	{
		cv::Vec3f p = point_data.at<cv::Vec3f>(i);
		if (p[2] > 1.f)
		{
			float d = std::pow(fabs(p.dot(normal) + param[3]), 2);
			dist.push_back(d);
		}
		
	}

	std::cout << "the mse is: "<< cv::mean(dist) << std::endl;


	std::cout << "ransac fitting plane is: \n" << param[0] << " " << param[1] << " " << param[2] << " " << param[3] << std::endl;

	std::cout << "pca plane fitting" << std::endl;
	std::cout << *(plane.get_normal()) << " " << *(plane.get_normal() + 1) << " " << *(plane.get_normal() + 2) << " " << plane.get_d() << std::endl;
	
	
	return 0;
}