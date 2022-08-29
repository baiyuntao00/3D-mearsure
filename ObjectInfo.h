#pragma once
#include "mcamera.h"
#include <vector>
#include <Eigen/Core>
#include <thread>
using namespace std;
using namespace Eigen;
//
//
struct MPlane
{
public:
	MPlane() {};
	~MPlane() {};
	MPlane(mp3 v0, mp3 v1, mp3 v2);
	MPlane(float a, float b, float c, float d) :A(a), B(b), C(c), D(d) {}
	float A, B, C, D;///平面参数
	vector<int>data_index;//保存平面内的点云固态索引
};
//
struct MNeighbor
{
public:
	MNeighbor() {};
public:
	int center_index;
	vector<int>able_ne_fit;
	MPlane plane;
};
//
class MLine
{
public:
	MPlane plane1, plane2;
	vector<int>data_index;
	Vector3d dir3d;
	Vector2d dir2d;
	mp2 v0, v1;
	float a, b, c;///ax+by+c=0
	mp3 p0;///直线上一点
	mcamera cam;
public:
	MLine() {};
	MLine(MPlane p1, MPlane p2,mcamera c) :plane1(p1), plane2(p2),cam(c){};
	bool Get2DLine(mpc::Ptr cloud, const struct rs2_intrinsics *intrin);
	void GetLineDir();
	mp2 GetLineV1(cv::Mat& image, cv::Mat image_depth, MLine line, float len, float * pcenter, float&s);
};
///

class ObjectInfo
{
public:
	ObjectInfo(mcamera&);
	bool GetPointCloudPlane();
	void GetNeighorArray(vector<MNeighbor>&);
	bool GetNeighborPlane(vector<int>point_index, MNeighbor &);
	bool FitCloudPlane(vector<MNeighbor>&neighbor);
	void TabPlane();
	bool IdentificationLine();
	void TwiceFitPlane(/*MPointRGBCloud::Ptr &cloud_rgb*/);
	MPlane RANSAC(mpc, float, bool&);
public:
	vector<int>pointIndex;
	mpc pc;
	mpcrgb pcrgb;
	vector<MPlane>planeArray;
	MLine cubeLine[3];
	int initPointNum;
	mcamera cam;
	float line_len[3];
};

