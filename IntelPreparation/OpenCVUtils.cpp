#include "OpenCVUtils.h"
#include "RealSenseUtils.h"

void calibrate(int number_of_calibration_images)
{
	VideoCapture cap(0);
	Mat image;
	cap >> image;
	Rect roi;
	roi.x = 0;
	roi.x = 0;
	roi.width = image.size().width / 2;
	roi.height = image.size().height;
	int numBoards = number_of_calibration_images;
	int numCornersHor = 9;
	int numCornersVer = 6;
	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);
	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;
	vector<Point2f> corners;
	int successes = 0;
	vector<Point3f> obj;

	for (int j = 0; j < numSquares; j++)
		obj.push_back(Point3f((float)(j / numCornersHor), (float)(j%numCornersHor), 0.0f));

	Mat gray_image;
	while (successes < numBoards)
	{
		cap >> image;
		//image = image(roi);
		cvtColor(image, gray_image, CV_BGR2GRAY);
		bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found)
		{
			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_sz, corners, found);
		}
		imshow("Gray Image", gray_image);
		int key = waitKey(1);
		if (key == 13 && found)
		{
			image_points.push_back(corners);
			object_points.push_back(obj);
			cout << "Calibration Image Stored" << endl;
			successes++;
			if (successes >= numBoards)
				break;
		}
	}
	destroyAllWindows();
	//Calibrate
	int flag = 0;
	flag |= fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	flag |= fisheye::CALIB_CHECK_COND;
	flag |= fisheye::CALIB_FIX_SKEW;
	Mat K;
	Mat D;
	fisheye::calibrate(object_points, image_points, image.size(), K, D, noArray(), noArray(), flag, TermCriteria(3, 20, 1e-6));
	cout << K << endl;
	cout << D << endl;

	ofstream myfile("camera.txt");
	if (myfile.is_open()){
		myfile << K.at<double>(0, 0) << endl;
		myfile << K.at<double>(0, 1) << endl;
		myfile << K.at<double>(0, 2) << endl;
		myfile << K.at<double>(1, 0) << endl;
		myfile << K.at<double>(1, 1) << endl;
		myfile << K.at<double>(1, 2) << endl;
		myfile << K.at<double>(2, 0) << endl;
		myfile << K.at<double>(2, 1) << endl;
		myfile << K.at<double>(2, 2) << endl;
		myfile << D.at<double>(0, 0) << endl;
		myfile << D.at<double>(0, 1) << endl;
		myfile << D.at<double>(0, 2) << endl;
		myfile << D.at<double>(0, 3) << endl;
		myfile.close();
	}
	else{
		cout << "Unable to open file" << endl;
	}
	getchar();
	cap.release();
}


void ir_stereo_calibrate(int number_of_calibration_images){
	std::cout << "Initialize RealSense For Intrinsic Calibration" << std::endl;

	PXCCapture::Device * device = NULL;
	PXCSession * session = NULL;
	PXCSenseManager * sense_manager = init_real_sense(320, 240, &device, &session);

	int numBoards = number_of_calibration_images;
	int numCornersHor = 9;
	int numCornersVer = 6;
	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);

	vector<vector<Point2f>> left_image_points;
	vector<Point2f> left_corners;
	vector<vector<Point2f>> right_image_points;
	vector<Point2f> right_corners;
	int successes = 0;
	vector<vector<Point3f>> object_points;
	vector<Point3f> obj;

	
	for (int j = 0; j < numSquares; j++)
		obj.push_back(Point3f((float)(j / numCornersHor), (float)(j%numCornersHor), 0.0f));

	//Left
	//Mat left_gray_image;
	while (successes < numBoards)
	{
		if (sense_manager->AcquireFrame(true) >= PXC_STATUS_NO_ERROR){
			PXCCapture::Sample * sample = sense_manager->QuerySample();
			PXCImage * left_image = sample->left;
			PXCImage::ImageInfo left_info = left_image->QueryInfo();
			PXCImage::ImageData left_data;
			left_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_Y8, &left_data);
			short * leftpixels = (short *)left_data.planes[0];
			int leftpitch = left_data.pitches[0];
			left_image->ReleaseAccess(&left_data);
			IplImage* leftimg = cvCreateImageHeader(cvSize(left_info.width, left_info.height), 8, 1);
			cvSetData(leftimg, leftpixels, leftpitch);
			cv::Mat leftMat = cv::cvarrToMat(leftimg);
			PXCImage * right_image = sample->right;
			PXCImage::ImageInfo right_info = right_image->QueryInfo();
			PXCImage::ImageData right_data;
			right_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_Y8, &right_data);
			short * rightpixels = (short *)right_data.planes[0];
			int rightpitch = right_data.pitches[0];
			right_image->ReleaseAccess(&right_data);
			IplImage* rightimg = cvCreateImageHeader(cvSize(right_info.width, right_info.height), 8, 1);
			cvSetData(rightimg, rightpixels, rightpitch);
			cv::Mat rightMat = cv::cvarrToMat(rightimg);
			sense_manager->ReleaseFrame();
			//image = image(roi);
			//cvtColor(leftMat, left_gray_image, CV_Y82BGR);
			bool found_left = findChessboardCorners(leftMat, board_sz, left_corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
			if (found_left)
			{
				cornerSubPix(leftMat, left_corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				drawChessboardCorners(leftMat, board_sz, left_corners, found_left);
			}
			bool found_right = findChessboardCorners(rightMat, board_sz, right_corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
			if (found_right)
			{
				cornerSubPix(rightMat, right_corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				drawChessboardCorners(rightMat, board_sz, right_corners, found_right);
			}
			imshow("Left Image", leftMat);
			imshow("Right Image", rightMat);
			int key = waitKey(1);
			if (key == 13 && found_left && found_right)
			{
				left_image_points.push_back(left_corners);
				right_image_points.push_back(right_corners);
				object_points.push_back(obj);
				cout << "Calibration Image Stored" << endl;
				successes++;
				if (successes >= numBoards)
					break;
			}
		}
	}


	////Right
	//for (int j = 0; j < numSquares; j++)
	//	right_obj.push_back(Point3f((float)(j / numCornersHor), (float)(j%numCornersHor), 0.0f));

	////Mat right_gray_image;
	//while (right_successes < numBoards)
	//{
	//	if (sense_manager->AcquireFrame(true) >= PXC_STATUS_NO_ERROR){
	//		PXCCapture::Sample * sample = sense_manager->QuerySample();
	//		PXCImage * right_image = sample->right;
	//		PXCImage::ImageInfo right_info = right_image->QueryInfo();
	//		PXCImage::ImageData right_data;
	//		right_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_Y8, &right_data);
	//		short * rightpixels = (short *)right_data.planes[0];
	//		int rightpitch = right_data.pitches[0];
	//		right_image->ReleaseAccess(&right_data);
	//		IplImage* rightimg = cvCreateImageHeader(cvSize(right_info.width, right_info.height), 8, 1);
	//		cvSetData(rightimg, rightpixels, rightpitch);
	//		cv::Mat rightMat = cv::cvarrToMat(rightimg);
	//		sense_manager->ReleaseFrame();
	//		//image = image(roi);
	//		//cvtColor(rightMat, right_gray_image, CV_BGR2GRAY);
	//		bool found = findChessboardCorners(rightMat, board_sz, right_corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	//		if (found)
	//		{
	//			cornerSubPix(rightMat, right_corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
	//			drawChessboardCorners(rightMat, board_sz, right_corners, found);
	//		}
	//		imshow("Right Image", rightMat);
	//		int key = waitKey(1);
	//		if (key == 13 && found)
	//		{
	//			right_image_points.push_back(right_corners);
	//			right_object_points.push_back(right_obj);
	//			cout << "Calibration Image Stored" << endl;
	//			right_successes++;
	//			if (right_successes >= numBoards)
	//				break;
	//		}
	//	}
	//}

	destroyAllWindows();

	cout << "Performing left camera calibration" << endl;
	Size left_image_size(320, 240);
	Mat leftK;
	Mat leftD;
	int cal_flags_left = cv::CALIB_RATIONAL_MODEL + cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5 + cv::CALIB_FIX_K6;
	cv::calibrateCamera(object_points, left_image_points, left_image_size, leftK, leftD, noArray(), noArray(), cal_flags_left, TermCriteria(3, 20, 1e-6));
	Mat optimal_left_K = cv::getOptimalNewCameraMatrix(leftK, leftD, left_image_size, 1, left_image_size);
	cout << optimal_left_K << endl;
	cout << leftD << endl;
	vector<vector<Point2f>> undistorted_left_image_points;
	cv::Mat left_points_input(1, (int)left_image_points.size(),CV_32FC2);
	//cv::Mat undistorted_left_points_input();
	cv::undistortPoints(left_image_points, undistorted_left_image_points, optimal_left_K, leftD, noArray(), noArray());

	cout << "Performing right camera calibration" << endl;
	Size right_image_size(320, 240);
	Mat rightK;
	Mat rightD;
	int cal_flags_right = cv::CALIB_RATIONAL_MODEL + cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5 + cv::CALIB_FIX_K6;
	cv::calibrateCamera(object_points, right_image_points, right_image_size, rightK, rightD, noArray(), noArray(), cal_flags_right, TermCriteria(3, 20, 1e-6));
	Mat optimal_right_K = cv::getOptimalNewCameraMatrix(rightK, rightD,right_image_size,1,right_image_size);
	cout << optimal_right_K << endl;
	cout << rightD << endl;
	vector<vector<Point2f>> undistorted_right_image_points;
	//cv::undistortPoints(right_image_points, undistorted_right_image_points, optimal_right_K, rightD, noArray(), noArray());

	cout << "Performing stereo calibration" << endl;
	Mat E, F, R, T;
	//int calib_flags = CALIB_FIX_INTRINSIC + 0 + CALIB_FIX_K3;
	int calib_flags = cv::CALIB_RATIONAL_MODEL + cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5 + cv::CALIB_FIX_K6;
	cv::stereoCalibrate(object_points, undistorted_left_image_points, undistorted_right_image_points, optimal_left_K, leftD, optimal_right_K, rightD, left_image_size, R, T, E, F, calib_flags, TermCriteria(3, 20, 1e-6));
	
	cout << R << endl;
	cout << T << endl;
	cout << E << endl;
	cout << F << endl;

	cv::Mat R1, R2, P1, P2, Q;

	cv::stereoRectify(optimal_left_K, leftD, optimal_right_K, rightD, left_image_size, R, T, R1, R2, P1, P2, Q, CV_CALIB_ZERO_DISPARITY, 0);

	cout << R1 << endl;
	cout << R2 << endl;
	cout << P1 << endl;
	cout << P2 << endl;
	cout << Q << endl;

	ofstream stereo_file("ir_stereo_calibration.txt");
	if (stereo_file.is_open()){
		stereo_file << Q.at<double>(0, 0) << endl;
		stereo_file << Q.at<double>(0, 1) << endl;
		stereo_file << Q.at<double>(0, 2) << endl;
		stereo_file << Q.at<double>(0, 3) << endl;
		stereo_file << Q.at<double>(1, 0) << endl;
		stereo_file << Q.at<double>(1, 1) << endl;
		stereo_file << Q.at<double>(1, 2) << endl;
		stereo_file << Q.at<double>(1, 3) << endl;
		stereo_file << Q.at<double>(2, 0) << endl;
		stereo_file << Q.at<double>(2, 1) << endl;
		stereo_file << Q.at<double>(2, 2) << endl;
		stereo_file << Q.at<double>(2, 3) << endl;
		stereo_file << Q.at<double>(3, 0) << endl;
		stereo_file << Q.at<double>(3, 1) << endl;
		stereo_file << Q.at<double>(3, 2) << endl;
		stereo_file << Q.at<double>(3, 3) << endl;
		stereo_file.close();
	}
	else{
		cout << "Unable to open file" << endl;
	}



	//session->Release();
	//device->Release();
	//sense_manager->Release();
	//sense_manager->Close();
	


}