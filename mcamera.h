#pragma once
#include<librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
//
#include "math.h"
#include <random> 
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//#include <pcl/filters/conditional_removal.h>
//#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h> 
//
#include <pcl/ModelCoefficients.h> //ģ��ϵ��
#include <pcl/sample_consensus/method_types.h> //�������һ�����㷨 ��������
#include <pcl/sample_consensus/model_types.h> //�������һ�����㷨 ģ������
#include <pcl/segmentation/sac_segmentation.h> //�������һ�����㷨 �ָ��
//
#define ROUND(a) int(a+0.5)
#define GRAY(r,g,b)  (0.299*r + 0.587*g + 0.144*b)
#define IMG_WIDTH 640
#define FLOATMIN 1e-6
#define IMG_HEIGHT 480
typedef pcl::PointXY mp2;
typedef pcl::PointXYZ mp3;
typedef pcl::PointXYZRGB mp3rgb;
typedef pcl::PointCloud<mp3> mpc;
typedef pcl::PointCloud<mp3rgb> mpcrgb;

using namespace cv;
using namespace rs2;
class mcamera
{
public:
	mcamera() {};
	~mcamera();
	mcamera(int h, int w) :height(h), width(w) {};
	Mat depth_mat, color_mat;
	String name;
	int height, width;
	float depth_scale;
	rs2::pipeline pipe;
	mpc pc;
	int cax;///������������С
	int cay;
	float local_t = 20;
	rs2_intrinsics in_par;//����ڲ�
	rs2_intrinsics in_par_color;//����ڲ�
	rs2::align *align;
public:
	bool initCamera();
	bool getImage();
	void getPointCloud();
	bool showCenterArea();
};

