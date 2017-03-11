#ifndef RENDER_3D_H
#define RENDER_3D_H 1

#include <time.h>
#include <iostream>
#include <cmath>

#include "Camera.h"
#include "PointCloud.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

using namespace std;

#define PI 3.14159265358

GLFWwindow* initWindow(const int resX, const int resY, int pos_x, int pos_y, const char * title);

void idle(GLFWwindow* window);

void show(GLFWwindow* window, int n, GLfloat *vertices, GLfloat *colors, Camera cam);

void show_map(GLFWwindow* window, int n, GLfloat *vertices, GLfloat *colors);

void show_points(GLFWwindow* window, GlobalMap s, Camera camera, Camera kinect);

void show_location(GLFWwindow* window, int n, GLfloat *vertices, GLfloat *colors, Camera camera, Camera kinect, vector<Camera> kinect_pos);

int terminate_point_renderer(GLFWwindow* window);

#endif