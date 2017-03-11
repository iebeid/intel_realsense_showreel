#ifndef MATH_H
#define MATH_H 1

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>

using namespace std;
//Math utilitis
class MathUtils{
public:

	MathUtils(){ ; }
	~MathUtils(){ ; }
	// A function to 
	static float * get_rotation_matrix_glm(float angle_x, float angle_y, float angle_z);
	static float * get_rotation_matrix(float angle_x, float angle_y, float angle_z);


};

#endif