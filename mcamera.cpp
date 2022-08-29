#include "pch.h"
#include "mcamera.h"
//
mcamera::~mcamera()
{
	align = NULL;
	delete align;
}
bool mcamera::initCamera()
{
	context ctx;
	auto dev_list = ctx.query_devices();
	if (dev_list.size() == 0)
		return false;
	device dev = *dev_list.begin();
	name = dev.get_info(RS2_CAMERA_INFO_NAME);
	config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
	//补洞滤波
	hole_filling_filter hole_filter;
	hole_filter.set_option(RS2_OPTION_HOLES_FILL, 2);
	//创建数据管道
	rs2::pipeline_profile profile = pipe.start(cfg);
	auto sensor = profile.get_device().first<rs2::depth_sensor>();
	sensor.set_option(RS2_OPTION_VISUAL_PRESET, 0);
	//获取数据流
	auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();	//声明内参
	auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();	//声明内参
	////深度向彩色对齐
	rs2_stream align_to = RS2_STREAM_COLOR ;
	align = new rs2::align(align_to);
	//深度比例
	depth_scale = sensor.as<rs2::depth_sensor>().get_depth_scale();
	//获取内参color大，depth小
	in_par_color= color_stream.get_intrinsics();
	in_par = depth_stream.get_intrinsics();
	return true;
}
bool mcamera::getImage()
{
//
	frameset data = pipe.wait_for_frames();
	auto processed = align->process(data);
	rs2::depth_frame depth = processed.get_depth_frame();
	rs2::video_frame color = processed.get_color_frame();
	//
	//in_par =depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
	depth_mat = Mat(Size(width, height), CV_16UC1, (void*)depth.get_data(), Mat::AUTO_STEP);
	color_mat = Mat(Size(width, height), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
	//
	return showCenterArea();
}
void mcamera::getPointCloud()
{
	mp2 center;
	center.x = width / 2, center.y = height / 2;
	for (int i = center.y - cay; i <= center.y + cay; i += 2)
	{
		for (int j = center.x - cax; j <= center.x + cax; j += 2)
		{
			float p3d[3] = { 0 ,0, 0 };
			float p2d[2] = { j ,i };
			mp3 point;
			float depth_val = depth_mat.at<uint16_t>(i, j)* depth_scale;
			if (depth_val < (float)0.00001&&depth_val>1.0)
				continue;
			rs2_deproject_pixel_to_point(p3d, &in_par, p2d, depth_val);
			point.x = p3d[0];
			point.y = p3d[1];
			point.z = p3d[2];
			pc.push_back(point);
		}
	}
}
bool mcamera::showCenterArea()
{
	mp2 center;
	center.x = width / 2, center.y = height / 2;
	float center_depth = depth_mat.at<uint16_t>(center.y, center.x);
	int x = center.x, y = center.y;
	float x_temp_depth = center_depth;
	float y_temp_depth = center_depth;
	while (y > 0)
	{
		y--;
		float y_depth = depth_mat.at<uint16_t>(y, center.x);
		if (fabsf(y_depth - y_temp_depth) > local_t)
			break;
		y_temp_depth = y_depth;
	}
	while (x > 0)
	{
		x--;
		float x_depth = depth_mat.at<uint16_t>(center.y, x);
		if (fabsf(x_depth - x_temp_depth) > local_t)
			break;
		x_temp_depth = x_depth;
	}
	cax = center.x - x + 20, cay = center.y - y + 20;
	if (center.x - cax > 40 && center.y - cay > 40 && cax > 50 && cay > 50)
	{
		line(color_mat, cv::Point(center.x - cax, center.y - cay), cv::Point(center.x - cax, center.y + cay), Scalar(255, 255, 255), 1, CV_AA);
		line(color_mat, cv::Point(center.x - cax, center.y - cay), cv::Point(center.x + cax, center.y - cay), Scalar(255, 255, 255), 1, CV_AA);
		line(color_mat, cv::Point(center.x + cax, center.y - cay), cv::Point(center.x + cax, center.y + cay), Scalar(255, 255, 255), 1, CV_AA);
		line(color_mat, cv::Point(center.x + cax, center.y + cay), cv::Point(center.x - cax, center.y + cay), Scalar(255, 255, 255), 1, CV_AA);
		return true;
	}
	else
		return false;
}