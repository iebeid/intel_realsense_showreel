#ifndef RENDER_POINTS_H
#define RENDER_POINTS_H 1

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include <iostream>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/vector_angle.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "PointCloud.h"

using namespace std;
using namespace glm;

#define PI 3.14159265358

void keyboard_controls(GLFWwindow* window, int key, int scancode, int action, int mods);

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

void error_callback(int error, const char* description);

GLFWwindow* initWindow(const int resX, const int resY, const char * title);



void show(GLFWwindow* window, PointCloud s);

int terminate_point_renderer(GLFWwindow* window);

#endif