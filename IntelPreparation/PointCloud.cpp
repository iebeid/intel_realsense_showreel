
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

PointCloud::PointCloud(cv::Mat depth_frame, cv::Mat mapped_rgb_frame, double * Q, int depth_width, int depth_height,
	int depth_lower_threshold, int depth_upper_threshold, int point_cloud_resolution){
	vector<cv::Mat> bgr;
	cv::split(mapped_rgb_frame, bgr);
	int number_of_elements_pushed = 0;
	//mat4 Q_glm = glm::make_mat4(Q);
	//Matrix Q_mat(4,4,Q);
	cv::Mat Q_64F(4, 4, CV_64F, Q);
	//cout << Q_64F << endl;
	//cout << glm::to_string(Q_glm) << endl;
	for (int v = 0; v < depth_height; v = v + point_cloud_resolution){
		for (int u = 0; u < depth_width; u = u + point_cloud_resolution){
			int idx = v*depth_width + u;
			//int vertical_idx = (v + point_cloud_resolution) * depth_width + u;
			//int horizontal_idx = v * depth_width + (u + point_cloud_resolution);
			double depth_value = depth_frame.data[idx];
			//cout << depth_value << endl;
			if (depth_value != 0){
				//cout << (depth_value*0.001f) << endl;
				//ushort vertical_depth_value = depth_frame.data[vertical_idx];
				//ushort horizontal_depth_value = depth_frame.data[horizontal_idx];
				float red = bgr[0].data[idx];
				float green = bgr[1].data[idx];
				float blue = bgr[2].data[idx];

				

				//double vector_temp_array[4];
				//vector_temp_array[0] = (double)u;
				//vector_temp_array[1] = (double)v;
				//vector_temp_array[2] = (double)(depth_value);
				//vector_temp_array[3] = 1.0;

				//Matrix vector_temp(4, 1, vector_temp_array);

				//cout << vector_temp.val[0][0] << " , " << vector_temp.val[0][1] << " , " << vector_temp.val[0][2] << " , " << vector_temp.val[0][3] << endl;

				cv::Mat_<double> vec(4, 1);
				vec(0) = u;
				vec(1) = v;
				vec(2) = depth_value;
				vec(3) = 1;
				vec = Q_64F*vec;
				vec /= vec(3);
				//vec4 vec_tmp;
				//vec_tmp.x = (float)u;
				//vec_tmp.y = (float)v;
				//vec_tmp.z = (float)depth_value;
				//vec_tmp.w = 1.0f;


				//
				//

				//
				//vec_tmp = Q_glm*vec_tmp;
				//vec_tmp /= vec_tmp.z;

				//Matrix q_vec_tmp = Q_mat * vector_temp;
				//vector_temp = Q_mat * vector_temp;

				//Matrix normalized_vector_point = q_vec_tmp / vector_temp_array[3];
				//vector_temp /= vector_temp.val[0][3];
				
				//cout << normalized_vector_point << endl;
				Point3D p;
				p.color.red = red / 255;
				p.color.blue = blue / 255;
				p.color.green = green / 255;
				//p.position.x = -(float)normalized_vector_point.val[0][0];
				//p.position.y = (float)normalized_vector_point.val[0][1];
				//p.position.z = (float)normalized_vector_point.val[0][2];
				p.position.x = vec(0)/1000;
				p.position.y = -vec(1)/1000;
				p.position.z = vec(2)/1000;
				//cout << p.position.x << " , " << p.position.y << " , " << p.position.z << endl;
				p.distance_from_origin = sqrt(pow(p.position.x, 2) + pow(p.position.y, 2) + pow(p.position.z, 2));



				//if (p.distance_from_origin > 0.0f && (p.color.red != 0 && p.color.green != 0 && p.color.blue != 0)){
					this->points.push_back(p);
					number_of_elements_pushed++;
				//}
			}

		}
	}
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