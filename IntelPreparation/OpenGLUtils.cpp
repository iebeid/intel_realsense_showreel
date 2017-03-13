#include "OpenGLUtils.h"
#include "MathUtils.h"
#include "Camera.h"



bool lbutton_up, lbutton_down;
bool rbutton_up, rbutton_down;
float zoom_distance = -1.0f;
float camera_position_x = 0.25f;
float camera_position_y = -0.5f;
float camera_position_z = zoom_distance;
float camera_angle_x = 25.0f;
float camera_angle_y = 180.0f;
float camera_angle_z = 0.0f;
Camera camera;

double cursor_xpos, cursor_ypos;
float mouse_position_x, mouse_position_y, dx, dy, dPhi, dTheta;



struct line_vertex
{
	// GL_C4UB_V3F
	unsigned char r, g, b, a;
	float x, y, z;
};

line_vertex g_lineVertices[] =
{
	{ 255, 0, 0, 255, 0.0f, 0.0f, 0.0f }, // red   = +x Axis
	{ 255, 0, 0, 255, 0.2f, 0.0f, 0.0f },
	{ 0, 255, 0, 255, 0.0f, 0.0f, 0.0f }, // green = +y Axis
	{ 0, 255, 0, 255, 0.0f, 0.2f, 0.0f },
	{ 0, 0, 255, 255, 0.0f, 0.0f, 0.0f }, // blue  = +z Axis
	{ 0, 0, 255, 255, 0.0f, 0.0f, 0.2f }
};

//void keyboard_controls(GLFWwindow* window, int key, int scancode, int action, int mods)
//{
//	if (action == GLFW_PRESS){
//		if (key == GLFW_KEY_ESCAPE){
//			glfwSetWindowShouldClose(window, GL_TRUE);
//		}
//
//	}
//}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	//if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	//{
	//	mouse_position_x = (float)cursor_xpos;
	//	mouse_position_y = (float)cursor_ypos;
	//}
	//if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_MOUSE_DOWN)
	//{
	//	cout << cursor_xpos << ", " << cursor_ypos << endl;
	//	float dPhi = ((float)(mouse_position_y - cursor_ypos) / 300);
	//	float dTheta = ((float)(mouse_position_x - cursor_xpos) / 300);
	//	camera.rotate(-dTheta, dPhi);
	//}

	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		if (GLFW_PRESS == action){
			lbutton_down = true;
			lbutton_up = false;
		}
		else if (GLFW_RELEASE == action){
			lbutton_down = false;
			lbutton_up = true;
		}
	}
	if (button == GLFW_MOUSE_BUTTON_RIGHT) {
		if (GLFW_PRESS == action){
			rbutton_down = true;
			rbutton_up = false;
		}
		else if (GLFW_RELEASE == action){
			rbutton_down = false;
			rbutton_up = true;
		}
	}



	//if (rbutton_down){
	//	cout << "Right Pressed" << endl;
	//	glfwGetCursorPos(window, &cursor_xpos, &cursor_ypos);
	//	cout << cursor_xpos << ", " << cursor_ypos << endl;
	//}

	//if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
	//{
	//	cout << cursor_xpos << ", " << cursor_ypos << endl;
	//	float dx = ((float)(mouse_position_x - cursor_xpos));
	//	float dy = ((float)(mouse_position_y - cursor_ypos));
	//	camera.pan(-dx * 1.0f, dy * 1.0f);
	//	mouse_position_x = (float)cursor_xpos;
	//	mouse_position_y = (float)cursor_ypos;
	//}

}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	if (yoffset < 0){
		zoom_distance = zoom_distance - 0.15f;
	}
	if (yoffset > 0){
		zoom_distance = zoom_distance + 0.15f;
	}
}
//
//void cursor_position_callback(GLFWwindow* window, double xpos, double ypos){
//	cursor_xpos = xpos;
//	cursor_ypos = ypos;
//	//dx = cursor_xpos - mouse_position_x;
//	//dy = cursor_ypos - mouse_position_y;
//	
//}

void error_callback(int error, const char* description)
{
	cout << description << endl;
}

GLFWwindow* initWindow(const int resX, const int resY, const char * title)
{
	const GLubyte* renderer;
	const GLubyte* version;
	glfwSetErrorCallback(error_callback);
	glfwInit();

	glfwWindowHint(GLFW_SAMPLES, 4);
	GLFWwindow* window = glfwCreateWindow(resX, resY, title, NULL, NULL);
	if (window == NULL)
	{
		fprintf(stderr, "Failed to open GLFW window.\n");
		glfwTerminate();
		return NULL;
	}
	glfwMakeContextCurrent(window);
	//glfwSetKeyCallback(window, keyboard_controls);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	//glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetScrollCallback(window, scroll_callback);

	glewExperimental = GL_TRUE;
	glewInit();
	renderer = glGetString(GL_RENDERER);
	version = glGetString(GL_VERSION);
	printf("Renderer: %s\n", renderer);
	printf("OpenGL version supported %s\n", version);

	//glfwSetInputMode(window, GLFW_STICKY_MOUSE_BUTTONS, 1);
	//glfwSetInputMode(window, GLFW_CURSOR_NORMAL, 1);
	//printf("Renderer: %s\n", glGetString(GL_RENDERER));
	//printf("OpenGL version supported %s\n", glGetString(GL_VERSION));
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	mouse_position_x = 0.0f;
	mouse_position_y = 0.0f;
	dx = 0.0f;
	dy = 0.0f;
	dPhi = 0.0f;
	dTheta = 0.0f;

	float pos[3] = { camera_position_x, camera_position_y, camera_position_z };
	camera.set_position(pos);
	float * rot = MathUtils::get_rotation_matrix(camera_angle_x, camera_angle_y, camera_angle_z);
	camera.set_rotation(rot);

	return window;
}



void show(GLFWwindow* window, PointCloud s){
	glfwMakeContextCurrent(window);
	if (!glfwWindowShouldClose(window))
	{
		
		// Scale to window size
		GLint windowWidth, windowHeight;
		glfwGetWindowSize(window, &windowWidth, &windowHeight);
		glViewport(0, 0, windowWidth, windowHeight);
		// Draw stuff
		glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_PROJECTION_MATRIX);
		glLoadIdentity();
		gluPerspective(90, (double)windowWidth / (double)windowHeight, 0.1, 100);
		glMatrixMode(GL_MODELVIEW_MATRIX);
		glfwGetCursorPos(window, &cursor_xpos, &cursor_ypos);
		//Mouse right click pan camera
		if (rbutton_down == true && rbutton_up == false) {
			dx = ((float)(mouse_position_x - cursor_xpos))*0.0001f;
			dy = ((float)(mouse_position_y - cursor_ypos))*0.0001f;
			float * current_camera_position = camera.get_position();
			float camera_new_position_x = current_camera_position[0] - dx;
			float camera_new_position_y = current_camera_position[1] + dy;
			float new_position[3] = { camera_new_position_x, camera_new_position_y, camera_position_z };
			camera.set_position(new_position);
		}
		//Mouse scroll zoom camera
		float * current_camera_position = camera.get_position();
		float camera_new_z_position = zoom_distance;
		float new_position[3] = { current_camera_position[0], current_camera_position[1], camera_new_z_position };
		camera.set_position(new_position);
		//Mouse rotate camera
		if (lbutton_down == true && lbutton_up == false) {
			dPhi = ((float)(mouse_position_y - cursor_ypos) / 10);
			dTheta = ((float)(mouse_position_x - cursor_xpos) / 10);
			float * current_camera_rotation = camera.get_rotation();
			float theta_x_camera = atan2(current_camera_rotation[7], current_camera_rotation[8]);
			float theta_y_camera = atan2(-current_camera_rotation[6], sqrt(pow(current_camera_rotation[7], 2) + pow(current_camera_rotation[8], 2)));
			float theta_z_camera = atan2(current_camera_rotation[3], current_camera_rotation[0]);
			float angle_x_camera = (theta_x_camera*180.0f) / (float)PI;
			float angle_y_camera = (theta_y_camera*180.0f) / (float)PI;
			float angle_z_camera = (theta_z_camera*180.0f) / (float)PI;
			float camera_new_angle_x = angle_x_camera + dPhi;
			float camera_new_angle_y = angle_y_camera - dTheta;
			float * rot = MathUtils::get_rotation_matrix(camera_new_angle_x, camera_new_angle_y, angle_z_camera);
			camera.set_rotation(rot);
		}
		//Update mouse postions
		mouse_position_x = (float)cursor_xpos;
		mouse_position_y = (float)cursor_ypos;
		//Camera
		float theta_x_camera = atan2(camera.get_rotation()[7], camera.get_rotation()[8]);
		float theta_y_camera = atan2(-camera.get_rotation()[6], sqrt(pow(camera.get_rotation()[7], 2) + pow(camera.get_rotation()[8], 2)));
		float theta_z_camera = atan2(camera.get_rotation()[3], camera.get_rotation()[0]);
		float angle_x_camera = (theta_x_camera*180.0f) / (float)PI;
		float angle_y_camera = (theta_y_camera*180.0f) / (float)PI;
		float angle_z_camera = (theta_z_camera*180.0f) / (float)PI;
		float pos_x_camera = camera.get_position()[0];
		float pos_y_camera = camera.get_position()[1];
		float pos_z_camera = camera.get_position()[2];
		glTranslatef(pos_x_camera, pos_y_camera, pos_z_camera);
		glRotatef(angle_x_camera, 1.0f, 0.0f, 0.0f);
		glRotatef(angle_y_camera, 0.0f, 1.0f, 0.0f);
		glRotatef(angle_z_camera, 0.0f, 0.0f, 1.0f);
		//
		//for (int i = 0; i < s.models.size(); i++){
		Render rs = s.get_rendering_structures();
		GLsizei number_of_points = (GLsizei)s.point_cloud_size;
		//cout << number_of_points << endl;
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_COLOR_ARRAY);
			glVertexPointer(3, GL_FLOAT, 0, rs.vertices);
			glColorPointer(3, GL_FLOAT, 0, rs.colors);
			glDrawArrays(GL_POINTS, 0, number_of_points);
			glDisableClientState(GL_COLOR_ARRAY);
			glDisableClientState(GL_VERTEX_ARRAY);
		//}
		//Axis
		glInterleavedArrays(GL_C4UB_V3F, 0, g_lineVertices);
		glDrawArrays(GL_LINES, 0, 6);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}
}

int terminate_point_renderer(GLFWwindow* window)
{
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}