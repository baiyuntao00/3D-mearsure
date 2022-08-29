// pch.cpp: 与预编译标头对应的源文件

#include "pch.h"
float nerighbor_plane_t = 0.003;
float fit_plane_t = 0.010;
float one_line_t = 0.008;
float find_radius = 0.015;//待确定
float sample_rate = 55;
float planeNum = 3;
float two_fit_plane_t = 0.007;
float local_t = 20;
bool is_result = false;
char* error_txt = "1";
// 当使用预编译的头时，需要使用此源文件，编译才能成功。