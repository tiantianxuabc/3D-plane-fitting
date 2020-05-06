#pragma once
#include<iostream>
#include <assert.h>

#include "Eigen/Core"
#include "Eigen/Dense"



class PCA_Plane 
{
	
private:
	double center[3];
	double normal[3];
	double mse;
	double curvature;
	


	double sx, sy, sz, //sum of x/y/z
		sxx, syy, szz, //sum of xx/yy/zz
		sxy, syz, sxz; //sum of xy/yz/xz
	int N; //number of points

public:

	PCA_Plane() : sx(0), sy(0), sz(0),
		sxx(0), syy(0), szz(0),
		sxy(0), syz(0), sxz(0), N(0) , mse(0), curvature(0)
	{
		memset(center, 0, sizeof(center)/sizeof(double));
		memset(normal, 0, sizeof(center) / sizeof(double));		
	}

	//merge from two other Stats
	PCA_Plane(const PCA_Plane& a, const PCA_Plane& b) :
		sx(a.sx + b.sx), sy(a.sy + b.sy), sz(a.sz + b.sz),
		sxx(a.sxx + b.sxx), syy(a.syy + b.syy), szz(a.szz + b.szz),
		sxy(a.sxy + b.sxy), syz(a.syz + b.syz), sxz(a.sxz + b.sxz), N(a.N + b.N) {}

	void clear() 
	{
		sx = sy = sz = sxx = syy = szz = sxy = syz = sxz = 0;
		N = 0;
	}

	//push a new point (x,y,z) into this Stats
	void push(const double x, const double y, const double z)
	{
		sx += x; sy += y; sz += z;
		sxx += x * x; syy += y * y; szz += z * z;
		sxy += x * y; syz += y * z; sxz += x * z;
		++N;
	}

	//push a new Stats into this Stats
	void push(const PCA_Plane& other)
	{
		sx += other.sx; sy += other.sy; sz += other.sz;
		sxx += other.sxx; syy += other.syy; szz += other.szz;
		sxy += other.sxy; syz += other.syz; sxz += other.sxz;
		N += other.N;
	}

	//caller is responsible to ensure (x,y,z) was collected in this stats
	void pop(const double x, const double y, const double z)
	{
		if (N > 0)
		{
			sx -= x; sy -= y; sz -= z;
			sxx -= x * x; syy -= y * y; szz -= z * z;
			sxy -= x * y; syz -= y * z; sxz -= x * z;
			--N;
		}
	}

	//caller is responsible to ensure {other} were collected in this stats
	void pop(const PCA_Plane& other)
	{
		if(N > 0)
		{
			sx -= other.sx; sy -= other.sy; sz -= other.sz;
			sxx -= other.sxx; syy -= other.syy; szz -= other.szz;
			sxy -= other.sxy; syz -= other.syz; sxz -= other.sxz;
			N -= other.N;
		}
	}

	/**
	*  \brief PCA-based plane fitting
	*
	*  \param [center]    `center center of mass of the PlaneSeg
	*  \param [normal]    normal unit normal vector of the PlaneSeg (ensure normal.z>=0)
	*  \param [mse]       mse mean-square-error of the plane fitting
	*  \param [curvature] curvature 
	*/
	void compute() 
	{
		if(N > 4)
		{
			const double sc = ((double)1.0) / this->N;//this->ids.size();
													  //calc plane equation: center, normal and mse
			center[0] = sx * sc;
			center[1] = sy * sc;
			center[2] = sz * sc;
			double K[3][3] =
			{
				{ sxx - sx * sx*sc, sxy - sx * sy*sc, sxz - sx * sz*sc },
			{ 0,                syy - sy * sy*sc, syz - sy * sz*sc },
			{ 0,                0,                szz - sz * sz*sc }
			};
			K[1][0] = K[0][1]; K[2][0] = K[0][2]; K[2][1] = K[1][2];
		

			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Eigen::Map<Eigen::Matrix3d>(K[0], 3, 3));
			Eigen::VectorXd v = es.eigenvectors().col(0);

			float d = -(v[0] * center[0] + v[1] * center[1] + v[2] * center[2]);
			// Enforce normal orientation
			if (d > 0) {
				normal[0] = v[0];
				normal[1] = v[1];
				normal[2] = v[2];
			}
			else {
				normal[0] = -v[0];
				normal[1] = -v[1];
				normal[2] = -v[2];
			}
			mse = es.eigenvalues()[0] * sc;
			curvature = es.eigenvalues()[0] / (v[0] + v[1] + v[2]);
			std::cout << "the mse is: " << mse << std::endl;
		}
		else
		{
			center[0] = center[1] = center[2] = 0.;
			normal[0] = normal[1] = normal[2] = 0.;
			mse = 0.;
			curvature = 0.;
		}

		
	}
	double get_d()
	{
		return -1.f*(normal[0] * center[0] + normal[1] * center[1] + normal[2] * center[2]);
	}
	double get_mse()
	{
		return mse;
	}
	double get_curvature()
	{
		return curvature;
	}
	double get_count()
	{
		return N;
	}
	double* get_normal()
	{
		return normal;
	}
	double* get_center()
	{
		return center;
	}

	inline double normalSimilarity(const PCA_Plane& p) const
	{
		return std::abs(normal[0] * p.normal[0] +
			normal[1] * p.normal[1] +
			normal[2] * p.normal[2]);
	}
	inline double normalSimilarity(cv::Point3f p) const
	{
		return std::abs(normal[0] * p.x + normal[1] * p.y + normal[2] * p.z);
	}


	/**
	*  \brief signed distance between this plane and the point pt[3]
	*/
	inline double signedDist(const double pt[3]) const
	{
		return fabs(normal[0] * (pt[0] - center[0]) +
			normal[1] * (pt[1] - center[1]) +
			normal[2] * (pt[2] - center[2]));
	}

};