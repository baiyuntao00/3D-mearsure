#include "pch.h"
#include "ObjectInfo.h"


MPlane::MPlane(mp3 v0, mp3 v1, mp3 v2)
{
	float d = -1.0*v0.x*(v1.y*v2.z - v2.y*v1.z) - v1.x*(v2.y*v0.z - v0.y*v2.z) - v2.x*(v0.y*v1.z - v1.y*v0.z);
	float a = v0.y*(v1.z - v2.z) + v1.y*(v2.z - v0.z) + v2.y*(v0.z - v1.z);
	float b = v0.z*(v1.x - v2.x) + v1.z*(v2.x - v0.x) + v2.z*(v0.x - v1.x);
	float c = v0.x*(v1.y - v2.y) + v1.x*(v2.y - v0.y) + v2.x*(v0.y - v1.y);
	float Mag = sqrt(a*a + b * b + c * c);
	if (fabsf(Mag) < FLOATMIN)
		Mag = 1.0;
	this->A = a / Mag;///单位法向量x分量
	this->B = b / Mag;///单位法向量y分量
	this->C = c / Mag;///单位法向量z分量
	this->D = d / Mag;///原点到平面的距离
	///
}
//
bool IsOnePlane(MPlane p1, MPlane p2, float t)///根据平面的四个参数确定两个平面能否拟合新的品面
{
	float c1 = fabsf(p1.A*p2.A + p1.B*p2.B + p1.C*p2.C);
	float c2 = sqrt(p1.A*p1.A + p1.B*p1.B + p1.C*p1.C)*sqrt(p2.A*p2.A + p2.B*p2.B + p2.C*p2.C);
	float cos_theta = c1 / c2;
	float theta = acos(cos_theta)*180.0 / CV_PI;
	return theta < t;
}
//
bool IsInPlane(MPlane pl, mp3 v, float max_dist)
{
	///根据点到平面的距离判断点是否属于平面
	float dist = 1.0*fabsf(pl.A*v.x + pl.B*v.y + pl.C*v.z + pl.D) / sqrt((pl.A*pl.A + pl.B*pl.B + pl.C*pl.C));
	return dist < max_dist;
}
///
bool IsOneLine(mp3 v0, mp3 v1, mp3 v2)
{
	//xy:
	float k_xy_01 = (v1.y - v0.y) / (v1.x - v0.x);
	float k_xy_12 = (v2.y - v1.y) / (v2.x - v1.x);
	if (fabsf(k_xy_01 - k_xy_12) > FLOATMIN)
		return false;
	//xz:
	float k_xz_01 = (v1.z - v0.z) / (v1.x - v0.x);
	float k_xz_12 = (v2.z - v1.z) / (v2.x - v1.x);
	if (fabsf(k_xz_01 - k_xz_12) > FLOATMIN)
		return false;
	//yz:
	float k_yz_01 = (v1.y - v0.y) / (v1.z - v0.z);
	float k_yz_12 = (v2.y - v1.y) / (v2.z - v1.z);
	if (fabsf(k_yz_01 - k_yz_12) > FLOATMIN)
		return false;
	return true;
}//
float DistPointToPlane(mp3 p, MPlane f)
{
	return fabsf(f.A*p.x + f.B*p.y + f.C*p.z + f.D) / sqrt(f.A*f.A + f.B*f.B + f.C*f.C);
}
float DistBetweenPoint3D(float * p1, float * p2)
{
	return sqrt(pow(p1[0] - p2[0],2.f) 
		+ pow(p1[1] - p2[1], 2.f)
		+ pow(p1[2] - p2[2], 2.f));
}
float DistBetweenPoint3D(mp3  p1, mp3 p2)
{
	return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z));
}
	//
bool MLine::Get2DLine(mpc::Ptr cloud, const struct rs2_intrinsics *intrin)
{
	float average[3] = { 0,0,0 };
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (IsInPlane(this->plane1, cloud->points[i], one_line_t) && IsInPlane(this->plane2, cloud->points[i], one_line_t))
		{
			average[0] += cloud->points[i].x;
			average[1] += cloud->points[i].y;
			average[2] += cloud->points[i].z;
			this->data_index.push_back(i);
		}
	}
	if (this->data_index.size() < 1)
		return false;
	//求质心
	average[0] /= this->data_index.size();
	average[1] /= this->data_index.size();
	average[2] /= this->data_index.size();
	//根据三维方向求二维方向
	float p12d[2];
	float p22d[2];
	rs2_project_point_to_pixel(p12d, intrin, average);
	for (int i = 1; i < 10; i++)
	{
		float t = i;
		float p23d[3] = { average[0] + dir3d(0) * t,average[1] + dir3d(1) * t,average[2] + dir3d(2) * t };
		rs2_project_point_to_pixel(p22d, intrin, p23d);
		if (abs(p12d[0] - p22d[0]) > 5 || abs(p12d[1] - p22d[1]) > 5)
			break;
	}
	dir2d(0) = p12d[0] - p22d[0];
	dir2d(1)= p12d[1] - p22d[1];
	dir2d.normalize();
	this->p0.x = p12d[0];
	this->p0.y = p12d[1];
	this->a = dir2d(1);
	this->b = -dir2d(0);
	this->c = -this->p0.x*dir2d(1) + this->p0.y*dir2d(0);
	return true;
}

ObjectInfo::ObjectInfo(mcamera &ca)
{
	cam = ca;
	pc = ca.pc;
	copyPointCloud(pc, pcrgb);
	this->initPointNum = pc.size();
	for (int i = 0; i < this->initPointNum; i++)
	{
		pointIndex.push_back(i);
		pcrgb[i].r= 255;
		pcrgb[i].g = 255;
		pcrgb[i].b = 255;
	}
}

MPlane ObjectInfo::RANSAC(mpc gnpcloud, float threld_dist_val, bool&label)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //存储输出的模型的系数
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //存储内点，使用的点
	pcl::SACSegmentation<mp3> seg;
	seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型，检测平面
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(gnpcloud.size()*0.7); //设置方法【聚类或随机样本一致性】
	seg.setDistanceThreshold(threld_dist_val);
	seg.setInputCloud(gnpcloud.makeShared());
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() != gnpcloud.size())
		label = false;
	return MPlane(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
}
bool ObjectInfo::GetNeighborPlane(vector<int>point_index, MNeighbor &ne)
{
	mpc gnpcloud;
	for (int i = 0; i < point_index.size(); i++)
	{
		int index = point_index[i];
		gnpcloud.push_back(pc[index]);
	}
	bool label = true;
	ne.plane = RANSAC(gnpcloud, nerighbor_plane_t, label);
	if (!label)
		return false;
	return true;
}


void ObjectInfo::GetNeighorArray(vector<MNeighbor>&neighbor)
{
	//函数调用
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pc.makeShared());
	//
	vector<int> neighbor_index;//保存领域内索引
	vector<float> pointRadiusSquaredDistance;//存储距离
	//
	float radius = find_radius;//查找半径
	//获取点云中所有的邻域
	int sample_num = (cam.cax * cam.cay) / sample_rate;
	//cout << "采样数：" << sample_num << endl;
	default_random_engine dre0(666);
	uniform_int_distribution<int>d0(1, 255);
	////
	default_random_engine dre1(666);
	uniform_int_distribution<int>d1(0, initPointNum - 1);
	for (int pi = 0; pi < sample_num; pi++)
	{
		int centerindex = d1(dre1) % initPointNum - 1;
		//int centerindex = rand() % initPointNum - 1;
		//test
		int r = d0(dre0);
		int g = d0(dre0);
		int b = d0(dre0);
		//
		if (kdtree.radiusSearch(pc[centerindex], radius, neighbor_index, pointRadiusSquaredDistance) > 0)
		{
			if (neighbor_index.size() < 20)
				continue;
			MNeighbor ne;//创建邻域对象
			ne.center_index = centerindex;

			if (GetNeighborPlane(neighbor_index, ne))
			//	//加入邻域集
			{
				neighbor.push_back(ne);
				//test
				for (int i = 0; i < neighbor_index.size(); i++)
				{
					int index = neighbor_index[i];
					//test
					pcrgb[index].r = r;
					pcrgb[index].g = g;
					pcrgb[index].b = b;
				}
				//
			}
		}
	}
	/*pcl::visualization::PCLVisualizer viewer("cloud example");
	viewer.addPointCloud(pcrgb.makeShared());
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}*/
	//test
	/*if (neighbor.size() < 30)
	{ 
		pcl::visualization::PCLVisualizer viewer("cloud example");
		viewer.addPointCloud(pcrgb.makeShared());
		while (!viewer.wasStopped()) {
			viewer.spinOnce();
		}
	}*/
	

}
///
bool NerighborCom(const MNeighbor& p1, const MNeighbor& p2)
{
	return p1.able_ne_fit.size() > p2.able_ne_fit.size();
}
bool ObjectInfo::FitCloudPlane(vector<MNeighbor>&neighbor)
{
	int num = neighbor.size();
	vector<MNeighbor>neighbor_temp= neighbor;
	for (int i = 0; i < num; i++)
	{
		for (int j = 0; j < num; j++)
		{
			int index = neighbor[j].center_index;
			if (IsInPlane(neighbor[i].plane, pc[index], find_radius / 3))
			{
				neighbor[i].able_ne_fit.push_back(j);
			}
		}
	}
	float temp_lt = 0.0;
	//float max_threld = 0.0;
	int max_index = 0;
	for (int i = 0; i < 4; i++)
	{
		if (neighbor.size() < 1)
			return false;
		sort(neighbor.begin(), neighbor.end(), NerighborCom);
		for (int n = 1; n < neighbor.size(); n++)
		{
			vector<int>result;
			set_intersection(neighbor[0].able_ne_fit.begin(),
				neighbor[0].able_ne_fit.end(),
				neighbor[n].able_ne_fit.begin(),
				neighbor[n].able_ne_fit.end(),
				back_inserter(result));//求交集 
			if (result.size() != 0)
			{
				neighbor.erase(neighbor.begin() + n);
				n = n - 1;
			}
		}
		//使用点与点的平均距离筛出地面
		planeArray.push_back(neighbor[0].plane);
		float lt = 0.0;
		for (int net = 0; net < neighbor[0].able_ne_fit.size(); net++)
		{
			int index1 = neighbor[0].able_ne_fit[net];
			int index1_1 = neighbor_temp[index1].center_index;
			int index2 = neighbor[0].center_index;
			lt += DistBetweenPoint3D(pc[index1_1], pc[index2]);
		}
	
		lt /= neighbor[0].able_ne_fit.size();
		if (lt > temp_lt)
		{
			temp_lt = lt;
			max_index = planeArray.size() - 1;
		}
		neighbor.erase(neighbor.begin());
	}
	//根据地面平面索引删除平面
	planeArray.erase(planeArray.begin() + max_index);
	/*for (int i = 0; i < planeArray[0].data_index.size(); i++)
	{
		int index = planeArray[0].data_index[i];
		pcrgb[index].r = 255;
		pcrgb[index].g = 0;
		pcrgb[index].b = 0;
	}
	pcl::visualization::PCLVisualizer viewer("cloud example");
	viewer.addPointCloud(pcrgb.makeShared());
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}*/
	return true;
}

///
bool ObjectInfo::GetPointCloudPlane()
{
	vector<MNeighbor> neighbor;//收集邻域集，收集所有能够拟合平面的邻域
	///采样几个邻域
	GetNeighorArray(neighbor);
	if (neighbor.size() < 1)
	{
		error_txt = "warn:邻域数量过少(GetPointCloudPlane_01)";
		return false;
	}
	///识别平面
	if (!FitCloudPlane(neighbor))
	{
		error_txt = "warn:邻域数量过少(GetPointCloudPlane_02)";
		return false;
	}
	return true;
}

void ObjectInfo::TabPlane()
{
	for (int i = 0; i < 3; i++)
	{
		//tab plane
		for (int j = 0; j < planeArray[i].data_index.size(); j++)
		{
			int index = planeArray[i].data_index[j];
			float pixel[2];
			float point[3] = { cam.pc[index].x,cam.pc[index].y ,cam.pc[index].z };
			rs2_project_point_to_pixel(pixel,&cam.in_par, point);
			if (pixel[0] > 640 || pixel[0] < 0)
				int i = 0;
			if (pixel[1] > 480 || pixel[1] < 0)
				int i = 0;
			for (int t = 0; t < 3; t++)
			{
				if (t == i)
					cam.color_mat.at<cv::Vec3b>(pixel[1], pixel[0])[t] = 255;
				else
					cam.color_mat.at<cv::Vec3b>(pixel[1], pixel[0])[t] = 0;
			}
		}
	}
}
void ObjectInfo::TwiceFitPlane(/*MPointRGBCloud::Ptr &cloud_rgb*/)
{
	//test
	//default_random_engine dre(666);
	//uniform_int_distribution<int>d(1, 255);
	//
	for (int p = 0; p < planeArray.size(); p++)
	{
		//test
		/*int r = d(dre);
		int g = d(dre);
		int b = d(dre);*/
		//
		mpc pc_temp;
		for (int i = 0; i < pc.size(); i++)
		{
			if (IsInPlane(planeArray[p], pc[i], fit_plane_t))
			{
				//planeArray[p].data_index.push_back(i);
				pc_temp.push_back(pc[i]);
				//test
			/*	pcrgb[i].r = r;
				pcrgb[i].g = g;
				pcrgb[i].b = b;*/
				
			}
		}
		bool t;
		MPlane temp = RANSAC(pc_temp, two_fit_plane_t, t);
		planeArray[p].A = temp.A;
		planeArray[p].B = temp.B;
		planeArray[p].C = temp.C;
		planeArray[p].D = temp.D;
		for (int i = 0; i < pc.size(); i++)
		{
			if (IsInPlane(planeArray[p], pc[i], two_fit_plane_t*0.8))
			{
				planeArray[p].data_index.push_back(i);
				//pc_temp.push_back(pc[i]);
				//test
				/*pcrgb[i].r = r;
				pcrgb[i].g = g;
				pcrgb[i].b = b;*/

			}
		}
	}
	//test
	/*pcl::visualization::PCLVisualizer viewer("cloud example");
	viewer.addPointCloud(pcrgb.makeShared());
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}*/
	//
}
float GetLocalGrey(cv::Mat image, int x, int y)
{
	return GRAY(image.at<cv::Vec3b>(ROUND(y), ROUND(x))[2], image.at<cv::Vec3b>(ROUND(y), ROUND(x))[1], image.at<cv::Vec3b>(ROUND(y), ROUND(x))[0]);
}
mp2 MLine::GetLineV1(cv::Mat& image, cv::Mat image_depth, MLine line, float len, float * pcenter, float&s)
{
	mp2 startp;
	mp2 endp;
	startp.x = line.dir2d(0) * len*s + pcenter[0];
	startp.y = line.dir2d(1) * len*s + pcenter[1];
	//get rgb color
	float gray_start = GetLocalGrey(image, startp.x, startp.y);
	//get value of depth
	float start_depth = image_depth.at<uint16_t>(ROUND(startp.y), ROUND(startp.x))*cam.depth_scale;
	while (s < 2.10)
	{
		s += 0.005;
		endp.x = line.dir2d(0) * len*s + pcenter[0];
		endp.y = line.dir2d(1) * len*s + pcenter[1];
		if (ROUND(endp.y) > IMG_HEIGHT / 2 + cam.cay)
		{
			error_txt = "warn:交线异常(GetLineV1_01) \n";
			endp.y = IMG_HEIGHT / 2 - cam.cay;
			break;
		}
		if (ROUND(endp.y) < IMG_HEIGHT / 2 - cam.cay)
		{
			error_txt = "warn:交线异常(GetLineV1_02) \n";
			endp.y = IMG_HEIGHT / 2 - cam.cay;
			break;
		}
		float gray_end = GetLocalGrey(image, endp.x, endp.y);
		float end_depth = image_depth.at<uint16_t>(ROUND(endp.y), ROUND(endp.x))*cam.depth_scale;
		if (fabsf(start_depth - end_depth) > 0.1)
			break;
		if (fabsf(gray_start - gray_end) > 3.0)
			break;
		gray_start = gray_end;
		start_depth = end_depth;
	}
	if (s >= 2.10)
		s = 2.0;
	endp.x = line.dir2d(0) * len*s + pcenter[0];
	endp.y = line.dir2d(1) * len*s + pcenter[1];
	return endp;
}
void MLine::GetLineDir()
{
	dir3d(0) = plane1.B*plane2.C - plane2.B*plane1.C;
	dir3d(1) = plane2.A*plane1.C - plane1.A*plane2.C;
	dir3d(2) = plane2.B*plane1.A - plane1.B*plane2.A;
	dir3d.normalize();
}
bool ObjectInfo::IdentificationLine()
{
	cubeLine[0] = MLine(planeArray[0], planeArray[1],cam);
	cubeLine[1] = MLine(planeArray[0], planeArray[2],cam);
	cubeLine[2] = MLine(planeArray[1], planeArray[2],cam);
	for (int i = 0; i < 3; i++)
	{
		cubeLine[i].GetLineDir();
		if (!cubeLine[i].Get2DLine(pc.makeShared(), &cam.in_par))
		{
			error_txt = "error:交线识别失败(Get2DLine) \n";
			return false;
		}
	}
	float cross_point[2] = {0,0};
	//1号线段
	float temp_cross_point = cubeLine[0].a*cubeLine[1].b - cubeLine[1].a*cubeLine[0].b;
	cross_point[1] += (cubeLine[1].a*cubeLine[0].c - cubeLine[0].a * cubeLine[1].c) / temp_cross_point;
	cross_point[0] += (cubeLine[0].b*cubeLine[1].c - cubeLine[0].c * cubeLine[1].b) / temp_cross_point;
	//2号线段
	temp_cross_point = cubeLine[1].a*cubeLine[2].b - cubeLine[2].a*cubeLine[1].b;
	cross_point[1] += (cubeLine[2].a*cubeLine[1].c - cubeLine[1].a * cubeLine[2].c) / temp_cross_point;
	cross_point[0] += (cubeLine[1].b*cubeLine[2].c - cubeLine[1].c * cubeLine[2].b) / temp_cross_point;
	//3号线段
	temp_cross_point = cubeLine[2].a*cubeLine[0].b - cubeLine[0].a*cubeLine[2].b;
	cross_point[1] += (cubeLine[0].a*cubeLine[2].c - cubeLine[2].a * cubeLine[0].c) / temp_cross_point;
	cross_point[0] += (cubeLine[2].b*cubeLine[0].c - cubeLine[2].c * cubeLine[0].b) / temp_cross_point;
	cross_point[1] /= 3;
	cross_point[0] /= 3;
	for (int i = 0; i < 3; i++)
	{

		cubeLine[i].v0.x = cross_point[0];
		cubeLine[i].v0.y = cross_point[1];
		//
		Vector2d v_test(cubeLine[i].p0.x - cross_point[0], cubeLine[i].p0.y - cross_point[1]);
		v_test.normalize();
		if (v_test.dot(cubeLine[i].dir2d) < 0)
		{
			cubeLine[i].dir2d(0) = -cubeLine[i].dir2d(0);
			cubeLine[i].dir2d(1) = -cubeLine[i].dir2d(1);
		}

		//
		int max_label = 0;
		float  reference_len = 0;
		for (int j = 0; j < cubeLine[i].data_index.size(); j++)
		{
			int index = cubeLine[i].data_index[j];
			float v1[2];
			float vt[3] = { pc[index].x ,pc[index].y ,pc[index].z };
			rs2_project_point_to_pixel(v1, &cam.in_par, vt);
			int v1x = ROUND(v1[0]);
			int v1y = ROUND(v1[1]);
			float len = sqrt((cross_point[0] - v1x)*(cross_point[0] - v1x) + (cross_point[1] - v1y)*(cross_point[1] - v1y));
			if (len > reference_len)
			{
				reference_len = len;
				max_label = index;
			}
		}
		//小段起点位置2D
		float standstart = reference_len * 0.1;
		float standard_v0_2d[2];
		standard_v0_2d[0] = cubeLine[i].dir2d(0) * standstart + cross_point[0];
		standard_v0_2d[1] = cubeLine[i].dir2d(1) * standstart + cross_point[1];
		float standard_v0_2d_depth = cam.depth_mat.at<uint16_t>(ROUND(standard_v0_2d[1]), ROUND(standard_v0_2d[0]));
		float standard_v0_3d[3] = { 0,0,0 };
		//小段起点位置3D
		rs2_deproject_pixel_to_point(standard_v0_3d, &cam.in_par, standard_v0_2d, standard_v0_2d_depth *cam.depth_scale);
		//小段终点位置2D
		float standend = reference_len * 0.6;
		float standard_v1_2d[2];
		standard_v1_2d[0] = cubeLine[i].dir2d(0) * standend + cross_point[0];
		standard_v1_2d[1] = cubeLine[i].dir2d(1) * standend + cross_point[1];
		float standard_v1_2d_depth = cam.depth_mat.at<uint16_t>(ROUND(standard_v1_2d[1]), ROUND(standard_v1_2d[0]));
		float standard_v1_3d[3] = { 0,0,0 };
		//小段终点位置3D
		rs2_deproject_pixel_to_point(standard_v1_3d, &cam.in_par, standard_v1_2d, standard_v1_2d_depth *cam.depth_scale);
		//求处小段标准长，即认为在该段内没有发生光线衍射
		float length_standard_depth = DistBetweenPoint3D(standard_v0_3d, standard_v1_3d);
		//在棱的终端点邻域内寻找最佳点
		float standard_scale = 1.93;
		reference_len /= 2;
		cubeLine[i].v1 = cubeLine[i].GetLineV1(cam.color_mat, cam.depth_mat, cubeLine[i], reference_len, cross_point, standard_scale);
		//
		////v1
		//float rev12[2] = { cubeLine[i].v1.x,cubeLine[i].v1.y };
		//float re = cam.depth_mat.at<uint16_t>(ROUND(rev12[1]), ROUND(rev12[0]));
		//float rev13[3] = { 0,0,0};
		//rs2_deproject_pixel_to_point(rev13, &cam.in_par, rev12, re*cam.depth_scale);
		////v2
		//float rev22[2] = { cubeLine[i].v0.x,cubeLine[i].v0.y };
		//float re2 = cam.depth_mat.at<uint16_t>(ROUND(rev22[1]), ROUND(rev22[0]));
		//float rev23[3] = { 0,0,0 };
		//rs2_deproject_pixel_to_point(rev23, &cam.in_par, rev22, re2*cam.depth_scale);
		//line_len[i] = DistBetweenPoint3D(rev23, rev13);
		//输出结果
		//depth内参下的计算结果
		line_len[i] = standard_scale * length_standard_depth*0.38;
		//color内参
		rs2_deproject_pixel_to_point(standard_v0_3d, &cam.in_par_color, standard_v0_2d, standard_v0_2d_depth *cam.depth_scale);
		rs2_deproject_pixel_to_point(standard_v1_3d, &cam.in_par_color, standard_v1_2d, standard_v1_2d_depth *cam.depth_scale);
		float length_standard_color = DistBetweenPoint3D(standard_v0_3d, standard_v1_3d);
		line_len[i] += standard_scale * length_standard_color*0.62;
		//line_len[i] /= 2;
	}
	// 着色显示
	line(cam.color_mat, cv::Point(cubeLine[0].v0.x, cubeLine[0].v0.y), cv::Point(cubeLine[0].v1.x, cubeLine[0].v1.y), Scalar(255, 0, 0), 2, CV_AA);
	line(cam.color_mat, cv::Point(cubeLine[1].v0.x, cubeLine[1].v0.y), cv::Point(cubeLine[1].v1.x, cubeLine[1].v1.y), Scalar(0, 255, 0), 2, CV_AA);
	line(cam.color_mat, cv::Point(cubeLine[2].v0.x, cubeLine[2].v0.y), cv::Point(cubeLine[2].v1.x, cubeLine[2].v1.y), Scalar(0, 0, 255), 2, CV_AA);
	//test
	//imshow("opencv", cam.color_mat);
	//
	return true;
}