#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H 1

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

#undef APIENTRY

using namespace std;

struct PXCColor{
	float blue;
	float green;
	float red;
};

struct Point
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
	PointCloud(vector<Point> p);
	PointCloud(cv::Mat depth_frame, cv::Mat mapped_rgb_frame, int depth_width, int depth_height, int depth_lower_threshold, int depth_upper_threshold, int point_cloud_resolution);
	~PointCloud();

	void dump(const char * filename);
	Render get_rendering_structures();
	void terminate(Render rs);

public:

	std::vector<Point> points;
};

struct GlobalMap{
	vector<PointCloud> point_clouds;
};
#endif