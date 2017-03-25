#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H 1

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>
#include <pxccapturemanager.h>


#undef APIENTRY

using namespace std;

struct PXCColor{
	float blue;
	float green;
	float red;
};

struct Point3D
{
	glm::vec3 position;
	glm::vec3 normal_vector;
	PXCColor color;
	PXCColor normal_color;
	float distance_from_origin;
};

struct Render{
	GLfloat * vertices;
	GLfloat * colors;
	GLfloat * normal_colors;
};

class Transformation{
public:
	Transformation(){
		R = glm::mat3(1.0f);
		t = glm::vec3();
	}
public:
	glm::mat3 R;
	glm::vec3 t;
};

class PointCloud
{
public:

	PointCloud(){ ; }
	PointCloud(vector<Point3D> p);
	//PointCloud(cv::Mat depth_frame, cv::Mat mapped_rgb_frame, double * Q, int depth_width, int depth_height, int depth_lower_threshold, int depth_upper_threshold, int point_cloud_resolution);
	PointCloud::PointCloud(double * Q, PXCImage * rgb_frame, PXCImage * depth_frame, PXCImage * mapped_rgb_frame, PXCSenseManager * sense_manager, PXCProjection * projection, PXCCapture::Device * device, int depth_threshold, int point_cloud_resolution);
	~PointCloud();

	void dump(const char * filename);
	void reprojectKinectDepth3D(cv::Mat& src, cv::Mat& dest, const double focal_length, cv::Point2d imageCenter);
	Render get_rendering_structures();
	void terminate(Render rs);

public:

	std::vector<Point3D> points;
	int point_cloud_size;
};

struct GlobalMap{
	vector<PointCloud> point_clouds;
};
#endif