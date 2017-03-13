

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <future>
#include <iterator>

#include <pxcsensemanager.h>

#include <opencv2/opencv.hpp>

#include <opencv2/core/ocl.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <nana/gui.hpp>
#include <nana/gui/widgets/label.hpp>
#include <nana/gui/widgets/button.hpp>
#include <nana/gui/widgets/slider.hpp>

#include "App.h"
#include "Queue.h"
#include "PointCloud.h"
#include "RealSenseUtils.h"
#include "OpenCVUtils.h"
#include "Render3D.h"
#include "OpenGLUtils.h"
#include "FPS.h"

//#include <nanogui/nanogui.h>

using namespace std;
using namespace nana;
//using namespace nanogui;

void filter_depth(){
	cout << "--------------------------" << endl;
	cout << "OpenCV version : " << CV_VERSION << endl;
	cout << "--------------------------" << endl;
	if (!ocl::haveOpenCL())
	{
		cout << "OpenCL is not avaiable..." << endl;
	}
	ocl::Context context;
	if (!context.create(cv::ocl::Device::TYPE_GPU))
	{
		cout << "Failed creating the context..." << endl;
	}
	cout << context.ndevices() << " GPU devices are detected." << endl;
	for (int i = 0; i < context.ndevices(); i++)
	{
		ocl::Device device = context.device(i);
		cout << "name                 : " << device.name() << endl;
		cout << "available            : " << device.available() << endl;
		cout << "imageSupport         : " << device.imageSupport() << endl;
		cout << "OpenCL_C_Version     : " << device.OpenCL_C_Version() << endl;
		cout << endl;
	}
	ocl::Device(context.device(0));

	ifstream ifs("kernel.cl");
	string kernelSource((istreambuf_iterator<char>(ifs)), istreambuf_iterator<char>());
	ocl::ProgramSource programSource(kernelSource);
	String errmsg;
	String buildopt = cv::format("-D dstT=float"); // "-D dstT=float"
	ocl::Program program = context.getProg(programSource, buildopt, errmsg);
	ocl::Kernel kernel("shift", program);


	PXCCapture::Device * device = NULL;
	PXCSession * session = NULL;
	PXCSenseManager * sense_manager = init_real_sense(320, 240, &device, &session);

	//PXCImage *mapped_color_to_depth = NULL;
	PXCCapture::Sample * sample = NULL;
	PXCImage *depth_image = NULL;
	FPS fps_counter;
	while (sense_manager->AcquireFrame(true) >= PXC_STATUS_NO_ERROR){
		fps_counter.start_fps_counter();
		sample = sense_manager->QuerySample();


		depth_image = sample->depth;
		PXCImage::ImageInfo depth_info = depth_image->QueryInfo();
		PXCImage::ImageData depth_data;
		depth_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &depth_data);
		short * dpixels = (short *)depth_data.planes[0];
		//cout << dpixels[122]*0.001 << " , " << dpixels[222]*0.001 << " , " << dpixels[512]*0.001 << endl;
		int dpitch = depth_data.pitches[0];
		depth_image->ReleaseAccess(&depth_data);

		cv::Mat depthBufferMat(depth_info.height, depth_info.width, CV_16UC1, dpixels, dpitch);
		cv::Mat depthMat(depth_info.height, depth_info.width, CV_8UC1);
		//cv::Mat depthMatValues(depth_info.height, depth_info.width, CV_16UC1);
		//depthBufferMat.convertTo(depthMat, CV_8UC1, -255.0f / maximum_depth, 255.0f);
		//depthBufferMat.convertTo(depthMatValues, CV_16UC1, -255.0f / 2500, 255.0f);
		depthBufferMat.convertTo(depthMat, CV_8UC1, -255.0f / 2500, 255.0f);

		//IplImage* depthimg = cvCreateImageHeader(cvSize(depth_info.width, depth_info.height), 16, 1);
		//cvSetData(depthimg, dpixels, dpitch);
		//cv::Mat depthMat = cv::cvarrToMat(depthimg);
		//cv::Mat depthMatConverted;
		//depthMat.convertTo(depthMatConverted, CV_8U, -255.0f / maximum_depth, 255.0f);
		cv::namedWindow("Depth");
		resizeWindow("Depth", 320, 240);
		cv::imshow("Depth", depthMat);



		//unsigned int channels = depthMat.channels();
		//unsigned int depth = depthMat.depth();

		UMat ocl_src;
		UMat ocl_dst;

		ocl_src = depthMat.getUMat(ACCESS_READ, USAGE_ALLOCATE_DEVICE_MEMORY);
		ocl_dst = UMat(depthMat.size(), depthMat.type(), ACCESS_WRITE, USAGE_ALLOCATE_DEVICE_MEMORY);



		ocl::Image2D image(ocl_src);
		float shift_x = -50.0f;
		float shift_y = -50.0f;
		

		std::size_t globalThreads[3] = { ocl_src.rows * ocl_src.step, 1, 1 };
		size_t localThreads[3] = { 256, 1, 1 };
		//std::vector<std::pair<size_t, const void *> > args;
		//args.push_back(std::make_pair(sizeof(uchar), (void *)&ocl_src));
		//args.push_back(std::make_pair(sizeof(uchar), (void *)&ocl_dst));

		////kernel.args(image, shift_x, shift_y, ocl::KernelArg::ReadWrite(umat_dst), 0, 0, gray_image.rows, gray_image.cols);
		//kernel.args(args);
		kernel.args(image, shift_x, shift_y, ocl::KernelArg::ReadWrite(ocl_dst), 0, 0, depthMat.rows, depthMat.cols);

		bool success = kernel.run(3, globalThreads, localThreads, true);
		if (!success){
			cout << "Failed running the kernel..." << endl;
			program.~Program();
			kernel.empty();
			kernel.~Kernel();
			getchar();
		}
		Mat mat_dst = ocl_dst.getMat(ACCESS_READ);
		namedWindow("Output", WINDOW_NORMAL);
		resizeWindow("Output", 320, 240);
		imshow("Output", mat_dst);


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

static void produce_point_cloud(Queue<PointCloud>& clouds, double * Q,int width, int height, int point_cloud_res, float maximum_depth){


	cout << "--------------------------" << endl;
	cout << "OpenCV version : " << CV_VERSION << endl;
	cout << "--------------------------" << endl;


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
		depth_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &depth_data);
		short * dpixels = (short *)depth_data.planes[0];
		//cout << dpixels[122]*0.001 << " , " << dpixels[222]*0.001 << " , " << dpixels[512]*0.001 << endl;
		int dpitch = depth_data.pitches[0];
		depth_image->ReleaseAccess(&depth_data);

		cv::Mat depthBufferMat(depth_info.height, depth_info.width, CV_16UC1, dpixels, dpitch);
		cv::Mat depthMat(depth_info.height, depth_info.width, CV_8UC1);
		//cv::Mat depthMatValues(depth_info.height, depth_info.width, CV_16UC1);
		//depthBufferMat.convertTo(depthMat, CV_8UC1, -255.0f / maximum_depth, 255.0f);
		//depthBufferMat.convertTo(depthMatValues, CV_16UC1, -255.0f / 2500, 255.0f);
		depthBufferMat.convertTo(depthMat, CV_8UC4, -255.0f / 2500, 255.0f);

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
		form fm;
		label lab{ fm, "<bold blue size=11>Intel RealSense Utilities</>" };
		lab.format(true);
		button btn{ fm, "Quit" };
		btn.events().click([&fm]{
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
				myfile.close();
			}
			else{
				cout << "Unable to open file" << endl;
			}
			int depth_width = 320;
			int depth_height = 240;
			int point_cloud_resolution = 1;
			float maximum_depth = 2500;
			Queue<PointCloud> clouds;
			using namespace std::placeholders;
			std::thread prod1(std::bind(produce_point_cloud, std::ref(clouds), std::ref(Q), std::ref(depth_width), std::ref(depth_height), std::ref(point_cloud_resolution), std::ref(maximum_depth)));
			std::thread consumer1(std::bind(consume_point_cloud, std::ref(clouds), std::ref(depth_width), std::ref(depth_height)));
			prod1.join();
			consumer1.join();
		});
		button btn_calibrate{ fm, "Calibrate" };
		btn_calibrate.events().click([&fm]{
			ir_stereo_calibrate(5);
		});
		button btn_filter_depth{ fm, "Filtered Depth" };
		btn_filter_depth.events().click([&fm]{
			filter_depth();
		});
		fm.div("vert <><<><weight=80% text><>><><weight=12<><button><>><>");
		fm["text"] << lab;
		fm["button"] << btn;
		fm["button"] << btn_calibrate;
		fm["button"] << btn_filter_depth;
		fm.collocate();
		fm.show();
		exec();
}

void App::terminate(){

}