#include "App.h"
#include "Queue.h"
#include "PointCloud.h"
#include "RealSenseUtils.h"
#include "OpenCVUtils.h"
#include "Render3D.h"
#include "OpenGLUtils.h"
#include "FPS.h"

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <future>
#include <iterator>

#include <pxcsensemanager.h>

#include <opencv2/opencv.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <nana/gui.hpp>
#include <nana/gui/widgets/label.hpp>
#include <nana/gui/widgets/button.hpp>

//#include <nanogui/nanogui.h>

using namespace std;
using namespace nana;
//using namespace nanogui;

static void produce_point_cloud(Queue<PointCloud>& clouds, double * Q,int width, int height, int point_cloud_res, float maximum_depth){
	std::cout << "Initialize RealSense" << std::endl;

	PXCCapture::Device * device = NULL;
	PXCSession * session = NULL;
	PXCSenseManager * sense_manager = init_real_sense(width, height, &device, &session);

	//PXCImage *mapped_color_to_depth = NULL;
	PXCCapture::Sample * sample = NULL;
	PXCImage *color_image = NULL;
	PXCImage *depth_image = NULL;
	//PXCImage *left_image = NULL;
	//PXCImage *right_image = NULL;
	PXCImage *mapped_color_to_depth_image = NULL;

	FPS fps_counter;


	//Define a form.
	form fm;

	//Define a label and display a text.
	label lab{ fm, "Hello, <bold blue size=16>Nana C++ Library</>" };
	lab.format(true);

	//Define a button and answer the click event.
	button btn{ fm, "Quit" };
	btn.events().click([&fm]{
		fm.close();
	});

	button btn_calibrate{ fm, "Calibrate" };
	btn_calibrate.events().click([&fm]{
		ir_stereo_calibrate(20);
	});


	

	//Layout management
	fm.div("vert <><<><weight=80% text><>><><weight=24<><button><>><>");
	fm["text"] << lab;
	fm["button"] << btn;
	fm["button"] << btn_calibrate;
	fm.collocate();

	//Show the form
	fm.show();

	//Start to event loop process, it blocks until the form is closed.
	exec();

	while (sense_manager->AcquireFrame(true) >= PXC_STATUS_NO_ERROR){
		fps_counter.start_fps_counter();
		sample = sense_manager->QuerySample();

		color_image = sample->color;
		PXCImage::ImageInfo color_info = color_image->QueryInfo();
		PXCImage::ImageData color_data;
		color_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &color_data);
		uchar * cpixels = (uchar *)color_data.planes[0];
		int cpitch = color_data.pitches[0];
		color_image->ReleaseAccess(&color_data);
		//IplImage* colorimg = cvCreateImageHeader(cvSize(color_info.width, color_info.height), 8, 4);
		//cvSetData(colorimg, cpixels, cpitch);
		//cv::Mat colorMat = cv::cvarrToMat(colorimg);
		//colorMat.convertTo(colorMat, CV_8UC4);

		cv::Mat colorMat(color_info.height, color_info.width, CV_8UC4, cpixels, cpitch);

		cv::namedWindow("Color");
		cv::imshow("Color", colorMat);


		depth_image = sample->depth;
		PXCImage::ImageInfo depth_info = depth_image->QueryInfo();
		PXCImage::ImageData depth_data;
		depth_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH_F32, &depth_data);
		float * dpixels = (float *)depth_data.planes[0];
		//cout << dpixels[122]*0.001 << " , " << dpixels[222]*0.001 << " , " << dpixels[512]*0.001 << endl;
		int dpitch = depth_data.pitches[0];
		depth_image->ReleaseAccess(&depth_data);

		cv::Mat depthBufferMat(depth_info.height, depth_info.width, CV_32F, dpixels, dpitch);
		cv::Mat depthMat(depth_info.height, depth_info.width, CV_8UC1);
		depthBufferMat.convertTo(depthMat, CV_8UC1, -255.0f / maximum_depth, 255.0f);

		//IplImage* depthimg = cvCreateImageHeader(cvSize(depth_info.width, depth_info.height), 16, 1);
		//cvSetData(depthimg, dpixels, dpitch);
		//cv::Mat depthMat = cv::cvarrToMat(depthimg);
		//cv::Mat depthMatConverted;
		//depthMat.convertTo(depthMatConverted, CV_8U, -255.0f / maximum_depth, 255.0f);
		cv::namedWindow("Depth");
		cv::imshow("Depth", depthMat);

		mapped_color_to_depth_image = map_color_to_depth(depth_image, color_image, session);

		
		PXCImage::ImageInfo mapped_info = mapped_color_to_depth_image->QueryInfo();
		PXCImage::ImageData mapped_data;
		mapped_color_to_depth_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &mapped_data);
		uchar * mapped_pixels = (uchar *)mapped_data.planes[0];
		int mapped_pitch = mapped_data.pitches[0];
		mapped_color_to_depth_image->ReleaseAccess(&mapped_data);
		//IplImage* mappedimg = cvCreateImageHeader(cvSize(mapped_info.width, mapped_info.height), 8, 4);
		//cvSetData(mappedimg, mapped_pixels, mapped_pitch);
		//cv::Mat mappedMat = cv::cvarrToMat(mappedimg);
		//mappedMat.convertTo(mappedMat, CV_8UC4);

		cv::Mat mappedMat(mapped_info.height, mapped_info.width, CV_8UC4, mapped_pixels, mapped_pitch);
		cv::namedWindow("Mapped Color to Depth");
		cv::imshow("Mapped Color to Depth", mappedMat);


		PointCloud current_point_cloud(depthBufferMat, mappedMat, Q, width, height, 0, (int)maximum_depth, point_cloud_res);

		

		clouds.push(current_point_cloud);

		sense_manager->ReleaseFrame();
		if (cv::waitKey(1) == 27){
			break;
		}
		fps_counter.end_fps_counter();
		fps_counter.print_fps();
	}
	session->Release();
	device->Release();

	sense_manager->Release();
	sense_manager->Close();

}

static void consume_point_cloud(Queue<PointCloud>& clouds, int width, int height){


	GLFWwindow* window = initWindow(800, 600, "Vertex Cloud");
	GLFWwindow* points_window = initWindow(width, height, 100, 800, "RealSense Point Cloud");
	GLFWwindow* normals_window = initWindow(width, height, 400, 800, "RealSense Normals");
	//Render Loop
	while (1){
		PointCloud point_cloud_object = clouds.pop();
		//cout << template_point_cloud.point_cloud_size << endl;
		//Render rs = template_point_cloud.get_rendering_structures();
		//cout << rs.vertices[10] << endl;
		Render ref_rs = point_cloud_object.get_rendering_structures();
		show_map(points_window, (int)point_cloud_object.points.size(), ref_rs.vertices, ref_rs.colors);
		show_map(normals_window, (int)point_cloud_object.points.size(), ref_rs.vertices, ref_rs.normal_colors);
		show(window, point_cloud_object);

	}
		
	terminate_point_renderer(window);


	//GLFWwindow* points_window = initWindow(width, height, 100, 800, "RealSense Point Cloud");
	//GLFWwindow* normals_window = initWindow(width, height, 400, 800, "RealSense Normals");
	//while (1){
	//	PointCloud point_cloud_object = clouds.pop();
	//	Render ref_rs = point_cloud_object.get_rendering_structures();
	//	show_map(points_window, (int)point_cloud_object.points.size(), ref_rs.vertices, ref_rs.colors);
	//	show_map(normals_window, (int)point_cloud_object.points.size(), ref_rs.vertices, ref_rs.normal_colors);
	//}
}

void App::run(){
	cout << "--------------------------" << endl;
	cout << "OpenCV version : " << CV_VERSION << endl;
	cout << "--------------------------" << endl;


	//nanogui::init();
	//Screen *screen = new Screen(Vector2i(300, 300), "");
	//bool enabled = true;
	//FormHelper *gui = new FormHelper(screen);
	//nanogui::ref<Window> window = gui->addWindow(Eigen::Vector2i(50, 100), "Control Panel");
	//gui->addButton("PointCloud Generation", [](){

		//cv::Mat Q;
	double * Q = new double[16];
		string line;
		ifstream myfile("ir_stereo_calibration.txt");
		if (myfile.is_open())
		{
			
			int i = 0;
			while (getline(myfile, line))
			{
				Q[i] = stod(line);
				i++;
			}
			//Q = cv::Mat(Size(4, 4), CV_64F, &qvalues);
			myfile.close();
		}
		else{
			cout << "Unable to open file" << endl;
		}
		//cout << "----------------------" << endl;
		//cout << Q << endl;
		//cout << "----------------------" << endl;

		int depth_width = 320;
		int depth_height = 240;
		int point_cloud_resolution = 1;
		float maximum_depth = 1500;
		Queue<PointCloud> clouds;
		using namespace std::placeholders;
		std::thread prod1(std::bind(produce_point_cloud, std::ref(clouds), std::ref(Q), std::ref(depth_width), std::ref(depth_height), std::ref(point_cloud_resolution), std::ref(maximum_depth)));
		std::thread consumer1(std::bind(consume_point_cloud, std::ref(clouds), std::ref(depth_width), std::ref(depth_height)));
		prod1.join();
		consumer1.join();


	//});
	//gui->addButton("Calibrate Stereo IR Camera", [](){
	//	ir_stereo_calibrate(10);
	//});

	//screen->setVisible(true);
	//screen->performLayout();
	//window->center();
	//nanogui::mainloop();
	//nanogui::shutdown();


}

void App::terminate(){

}