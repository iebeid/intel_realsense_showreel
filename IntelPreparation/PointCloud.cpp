
#include "PointCloud.h"
//#include "Vector.h"

#include <memory>
#include <vector>
#include <iostream>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include "Matrix.h"
using namespace glm;

PointCloud::~PointCloud(){
	points.clear();
	points.shrink_to_fit();
}

//void PointCloud::reprojectKinectDepth3D(cv::Mat& src, cv::Mat& dest, const double focal_length, cv::Point2d imageCenter = cv::Point2d(-1, -1))
//{
//	if (dest.empty())dest = cv::Mat::zeros(src.size().area(), 1, CV_32FC3);
//
//	const double bigZ = 10000.;
//	const double hw = (src.cols - 1)*0.5;
//	const double hh = (src.rows - 1)*0.5;
//	if (imageCenter.x == -1 && imageCenter.y == -1)imageCenter = cv::Point2d(hw, hh);
//	double ifocal_length = 1.0 / focal_length;
//#pragma omp parallel for
//	for (int j = 0; j<src.rows; j++)
//	{
//		float* data = dest.ptr<float>(j*src.cols);
//		unsigned short* s = src.ptr<unsigned short>(j);
//		for (int i = 0; i<src.cols; i++)
//		{
//			data[0] = *s*ifocal_length*(i - imageCenter.x);
//			data[1] = *s*ifocal_length*(j - imageCenter.y);
//			if (*s == 0)data[2] = bigZ;
//			else data[2] = *s;
//
//			data += 3;
//			s++;
//		}
//	}
//}

//http://opencv.jp/opencv2-x-samples/point-cloud-rendering
//http://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
//http://stackoverflow.com/questions/22418846/reprojectimageto3d-in-opencv
PointCloud::PointCloud(cv::Mat depth_frame, cv::Mat mapped_rgb_frame, double * Q, int depth_width, int depth_height,
	int depth_lower_threshold, int depth_upper_threshold, int point_cloud_resolution){
	vector<cv::Mat> bgr;
	cv::split(mapped_rgb_frame, bgr);

	//cv::initUndistortRectifyMap(left_cam_matrix, left_dist_coeffs, R1, P1, frame_size, CV_32FC1, left_undist_rect_map_x, left_undist_rect_map_y);


	int number_of_elements_pushed = 0;
	//mat4 Q_glm = glm::make_mat4(Q);
	//Matrix Q_mat(4,4,Q);
	cv::Mat Q_64F(4, 4, CV_64F, Q);
	//cout << Q_64F << endl;
	cv::Mat Q_32F(4, 4, CV_32F);
	Q_64F.convertTo(Q_32F,CV_32F);
	//cout << Q_32F << endl;

	cv::Point2d imageCenter = cv::Point2d(-1, -1);

	//cv::Mat xyz;
	

	//const double bigZ = 10000.;
	const double hw = (depth_frame.cols - 1)*0.5;
	const double hh = (depth_frame.rows - 1)*0.5;
	if (imageCenter.x == -1 && imageCenter.y == -1)imageCenter = cv::Point2d(hw, hh);
	double ifocal_length = 1.0 / 329.874;
#pragma omp parallel for
	for (int j = 0; j<depth_frame.rows; j++)
	{
		//float* data = dest.ptr<float>(j*src.cols);
		short* s = depth_frame.ptr<short>(j);
		for (int i = 0; i<depth_frame.cols; i++)
		{
			int idx = j*depth_width + i;
			//		for (int u = 0; u < depth_width;
			float red = bgr[0].data[idx];
			float green = bgr[1].data[idx];
			float blue = bgr[2].data[idx];
			Point3D p;
			p.color.red = red / 255;
			p.color.blue = blue / 255;
			p.color.green = green / 255;
			//p.position.x = -(float)normalized_vector_point.val[0][0];
			//p.position.y = (float)normalized_vector_point.val[0][1];
			//p.position.z = (float)normalized_vector_point.val[0][2];
			//data[0] = *s*ifocal_length*(i - imageCenter.x);
			//data[1] = *s*ifocal_length*(j - imageCenter.y);

			p.position.x = -(float)(*s*ifocal_length*(i - imageCenter.x))/1000;
			p.position.y = -(float)(*s*ifocal_length*(j - imageCenter.y))/1000;
			 
			 //if (*s == 0)p.position.z = (bigZ)/1000;
			 p.position.z = (float)((*s)/1000);
			//cout << p.position.x << " , " << p.position.y << " , " << p.position.z << endl;
			p.distance_from_origin = sqrt(pow(p.position.x, 2) + pow(p.position.y, 2) + pow(p.position.z, 2));



			if (p.distance_from_origin > 0.0f && (p.color.red != 0 && p.color.green != 0 && p.color.blue != 0)){
				this->points.push_back(p);
				number_of_elements_pushed++;
			}
			s++;
		}
	}



//	//cout << glm::to_string(Q_glm) << endl;
//#pragma omp parallel for
//	for (int v = 0; v < depth_height; v = v + point_cloud_resolution){
//		//float * depth_value = depth_frame.data[idx];
//		float * depth_value = depth_frame.ptr<float>(v);
//		for (int u = 0; u < depth_width; u = u + point_cloud_resolution){
//			int idx = v*depth_width + u;
//			//int vertical_idx = (v + point_cloud_resolution) * depth_width + u;
//			//int horizontal_idx = v * depth_width + (u + point_cloud_resolution);
//			float depth_value = depth_frame.data[idx];
//			//cout << depth_value << endl;
//			if (depth_value > 0){
//				//cout << (depth_value*0.001f) << endl;
//				//ushort vertical_depth_value = depth_frame.data[vertical_idx];
//				//ushort horizontal_depth_value = depth_frame.data[horizontal_idx];
//				float red = bgr[0].data[idx];
//				float green = bgr[1].data[idx];
//				float blue = bgr[2].data[idx];
//
//				
//
//				//double vector_temp_array[4];
//				//vector_temp_array[0] = (double)u;
//				//vector_temp_array[1] = (double)v;
//				//vector_temp_array[2] = (double)(depth_value);
//				//vector_temp_array[3] = 1.0;
//
//				//Matrix vector_temp(4, 1, vector_temp_array);
//
//				//cout << vector_temp.val[0][0] << " , " << vector_temp.val[0][1] << " , " << vector_temp.val[0][2] << " , " << vector_temp.val[0][3] << endl;
//
//				//depth_value = (-255.0f / 2500)*depth_value + 255.0f;
//
//				cv::Mat_<float> vec(4, 1);
//				vec(0) = (float)u;
//				vec(1) = (float)v;
//				vec(2) = depth_value;
//				vec(3) = 1.0f;
//				vec = Q_32F*vec;
//				vec /= vec(3);
//				//vec4 vec_tmp;
//				//vec_tmp.x = (float)u;
//				//vec_tmp.y = (float)v;
//				//vec_tmp.z = (float)depth_value;
//				//vec_tmp.w = 1.0f;
//
//
//				//
//				//
//
//				//
//				//vec_tmp = Q_glm*vec_tmp;
//				//vec_tmp /= vec_tmp.z;
//
//				//Matrix q_vec_tmp = Q_mat * vector_temp;
//				//vector_temp = Q_mat * vector_temp;
//
//				//Matrix normalized_vector_point = q_vec_tmp / vector_temp_array[3];
//				//vector_temp /= vector_temp.val[0][3];
//				
//				//cout << normalized_vector_point << endl;
//				Point3D p;
//				p.color.red = red / 255;
//				p.color.blue = blue / 255;
//				p.color.green = green / 255;
//				//p.position.x = -(float)normalized_vector_point.val[0][0];
//				//p.position.y = (float)normalized_vector_point.val[0][1];
//				//p.position.z = (float)normalized_vector_point.val[0][2];
//				p.position.x = (float)(-vec(0));
//				p.position.y = (float)(-vec(1));
//				p.position.z = (float)(vec(2));
//				//cout << p.position.x << " , " << p.position.y << " , " << p.position.z << endl;
//				p.distance_from_origin = sqrt(pow(p.position.x, 2) + pow(p.position.y, 2) + pow(p.position.z, 2));
//
//
//
//				if (p.distance_from_origin > 0.0f && (p.color.red != 0 && p.color.green != 0 && p.color.blue != 0)){
//					this->points.push_back(p);
//					number_of_elements_pushed++;
//				}
//			}
//
//		}
//	}
	this->points.resize(number_of_elements_pushed);
	this->point_cloud_size = number_of_elements_pushed;
}

PointCloud::PointCloud(vector<Point3D> p){
	this->points = p;
	memcpy(&this->points[0], &p[0], p.size()*sizeof(Point3D));
}

void PointCloud::dump(const char * filename){
	std::ofstream myfile(filename);
	if (myfile.is_open()){

		for (int i = 0; i < this->points.size(); i++){
			myfile << this->points[i].position.x << " " << this->points[i].position.y << "	" << this->points[i].position.z <<
				" " << this->points[i].color.red << " " << this->points[i].color.green << " " << this->points[i].color.blue <<
				" " << this->points[i].normal_vector.x << " " << this->points[i].normal_vector.y << " " << this->points[i].normal_vector.z << endl;
		}
		myfile.close();
	}
	else{
		std::cout << "Unable to open file" << std::endl;
	}
}

Render PointCloud::get_rendering_structures(){
	Render rs;
	rs.vertices = new float[this->points.size() * 3];
	rs.colors = new float[this->points.size() * 3];
	rs.normal_colors = new float[this->points.size() * 3];
	int t = 0;
	Point3D p;
	for (int j = 0; j < this->points.size(); j++)
	{
		p = this->points[j];
		rs.vertices[t] = (float)p.position.x;
		rs.vertices[t + 1] = (float)p.position.y;
		rs.vertices[t + 2] = (float)p.position.z;
		rs.colors[t] = (float)p.color.blue;
		rs.colors[t + 1] = (float)p.color.green;
		rs.colors[t + 2] = (float)p.color.red;
		rs.normal_colors[t] = (float)p.normal_color.red;
		rs.normal_colors[t + 1] = (float)p.normal_color.green;
		rs.normal_colors[t + 2] = (float)p.normal_color.blue;
		t = t + 3;
	}
	return rs;
}

void PointCloud::terminate(Render rs){
	free(rs.vertices);
	free(rs.colors);
	free(rs.normal_colors);
	this->~PointCloud();
}