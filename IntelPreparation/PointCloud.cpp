
#include "PointCloud.h"
//#include "Vector.h"

#include <memory>
#include <vector>
#include <iostream>
#include <vector>

#include <glm/glm.hpp>

using namespace glm;

PointCloud::~PointCloud(){
	points.clear();
	points.shrink_to_fit();
}

PointCloud::PointCloud(cv::Mat depth_frame, cv::Mat mapped_rgb_frame, int depth_width, int depth_height,
	int depth_lower_threshold, int depth_upper_threshold, int point_cloud_resolution){
	vector<cv::Mat> bgr;
	cv::split(mapped_rgb_frame, bgr);
	int number_of_elements_pushed = 0;
	for (int v = 0; v < depth_height; v = v + point_cloud_resolution){
		for (int u = 0; u < depth_width; u = u + point_cloud_resolution){
			int idx = v*depth_width + u;
			int vertical_idx = (v + point_cloud_resolution) * depth_width + u;
			int horizontal_idx = v * depth_width + (u + point_cloud_resolution);
			ushort depth_value = depth_frame.data[idx];

		}
	}
}

PointCloud::PointCloud(vector<Point> p){
	this->points = p;
	memcpy(&this->points[0], &p[0], p.size()*sizeof(Point));
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
	Point p;
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