#include "App.h"
#include "Queue.h"
#include "PointCloud.h"
#include "RealSenseUtils.h"
#include "FPS.h"

#include <pxcsensemanager.h>

#include <opencv2/opencv.hpp>

static void produce_point_cloud(Queue<PointCloud>& clouds, int width, int height, int point_cloud_res, int maximum_depth){
	std::cout << "Initialize RealSense" << std::endl;

	PXCCapture::Device * device = NULL;
	PXCSession * session = NULL;
	PXCSenseManager * sense_manager = init_real_sense(width, height, &device, &session);

	PXCImage *mapped_color_to_depth = NULL;
	PXCCapture::Sample * sample = NULL;
	PXCImage *color_image = NULL;
	PXCImage *depth_image = NULL;
	PXCImage *left_image = NULL;
	PXCImage *right_image = NULL;

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
		IplImage* colorimg = cvCreateImageHeader(cvSize(color_info.width, color_info.height), 8, 4);
		cvSetData(colorimg, cpixels, cpitch);
		cv::Mat colorMat = cv::cvarrToMat(colorimg);
		colorMat.convertTo(colorMat, CV_8UC4);
		cv::namedWindow("Color");
		cv::imshow("Color", colorMat);

		depth_image = sample->depth;
		PXCImage::ImageInfo depth_info = depth_image->QueryInfo();
		PXCImage::ImageData depth_data;
		depth_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &depth_data);
		short * dpixels = (short *)depth_data.planes[0];
		int dpitch = depth_data.pitches[0];
		depth_image->ReleaseAccess(&depth_data);
		IplImage* depthimg = cvCreateImageHeader(cvSize(depth_info.width, depth_info.height), 16, 1);
		cvSetData(depthimg, dpixels, dpitch);
		cv::Mat depthMat = cv::cvarrToMat(depthimg);
		cv::Mat depthMatConverted;
		depthMat.convertTo(depthMatConverted, CV_8U, -255.0f / 3000.0f, 255.0f);
		cv::namedWindow("Depth");
		cv::imshow("Depth", depthMatConverted);

		left_image = sample->left;
		PXCImage::ImageInfo left_info = left_image->QueryInfo();
		PXCImage::ImageData left_data;
		left_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_Y8, &left_data);
		short * leftpixels = (short *)left_data.planes[0];
		int leftpitch = left_data.pitches[0];
		left_image->ReleaseAccess(&left_data);
		IplImage* leftimg = cvCreateImageHeader(cvSize(left_info.width, left_info.height), 8, 1);
		cvSetData(leftimg, leftpixels, leftpitch);
		cv::Mat leftMat = cv::cvarrToMat(leftimg);
		cv::namedWindow("IR Left");
		cv::imshow("IR Left", leftMat);

		right_image = sample->right;
		PXCImage::ImageInfo right_info = right_image->QueryInfo();
		PXCImage::ImageData right_data;
		right_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_Y8, &right_data);
		short * rightpixels = (short *)right_data.planes[0];
		int rightpitch = right_data.pitches[0];
		right_image->ReleaseAccess(&right_data);
		IplImage* rightimg = cvCreateImageHeader(cvSize(right_info.width, right_info.height), 8, 1);
		cvSetData(rightimg, rightpixels, rightpitch);
		cv::Mat rightMat = cv::cvarrToMat(rightimg);
		cv::namedWindow("IR Right");
		cv::imshow("IR Right", rightMat);

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
}

void App::run(){
	int depth_width = 320;
	int depth_height = 240;
	int point_cloud_resolution = 2;
	int maximum_depth = 1500;
	Queue<PointCloud> clouds;
	using namespace std::placeholders;

	std::thread prod1(std::bind(produce_point_cloud, std::ref(clouds), std::ref(depth_width), std::ref(depth_height), std::ref(point_cloud_resolution), std::ref(maximum_depth)));
	std::thread consumer1(std::bind(consume_point_cloud, std::ref(clouds), std::ref(depth_width), std::ref(depth_height)));
	prod1.join();
	consumer1.join();
}

void App::terminate(){

}