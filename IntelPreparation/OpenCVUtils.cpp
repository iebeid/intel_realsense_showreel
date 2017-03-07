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

	
	PXCCapture::Sample * sample = NULL;
	PXCImage *left_image = NULL;
	PXCImage *right_image = NULL;

	int numBoards = number_of_calibration_images;
	int numCornersHor = 9;
	int numCornersVer = 6;
	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);

	vector<vector<Point3f>> left_object_points;
	vector<vector<Point2f>> left_image_points;
	vector<Point2f> left_corners;
	int left_successes = 0;
	vector<Point3f> left_obj;

	vector<vector<Point3f>> right_object_points;
	vector<vector<Point2f>> right_image_points;
	vector<Point2f> right_corners;
	int right_successes = 0;
	vector<Point3f> right_obj;

	//Left
	for (int j = 0; j < numSquares; j++)
		left_obj.push_back(Point3f((float)(j / numCornersHor), (float)(j%numCornersHor), 0.0f));

	Mat left_gray_image;
	while (left_successes < numBoards)
	{
		sample = sense_manager->QuerySample();
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
		sense_manager->ReleaseFrame();
		//image = image(roi);
		cvtColor(leftMat, left_gray_image, CV_BGR2GRAY);
		bool found = findChessboardCorners(leftMat, board_sz, left_corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found)
		{
			cornerSubPix(left_gray_image, left_corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(left_gray_image, board_sz, left_corners, found);
		}
		imshow("Gray Image", left_gray_image);
		int key = waitKey(1);
		if (key == 13 && found)
		{
			left_image_points.push_back(left_corners);
			left_object_points.push_back(left_obj);
			cout << "Calibration Image Stored" << endl;
			left_successes++;
			if (left_successes >= numBoards)
				break;
		}
	}


	//Right
	for (int j = 0; j < numSquares; j++)
		right_obj.push_back(Point3f((float)(j / numCornersHor), (float)(j%numCornersHor), 0.0f));

	Mat right_gray_image;
	while (right_successes < numBoards)
	{
		sample = sense_manager->QuerySample();
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
		sense_manager->ReleaseFrame();
		//image = image(roi);
		cvtColor(rightMat, right_gray_image, CV_BGR2GRAY);
		bool found = findChessboardCorners(rightMat, board_sz, right_corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found)
		{
			cornerSubPix(right_gray_image, right_corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(right_gray_image, board_sz, right_corners, found);
		}
		imshow("Gray Image", right_gray_image);
		int key = waitKey(1);
		if (key == 13 && found)
		{
			right_image_points.push_back(right_corners);
			right_object_points.push_back(right_obj);
			cout << "Calibration Image Stored" << endl;
			right_successes++;
			if (right_successes >= numBoards)
				break;
		}
	}

	session->Release();
	device->Release();
	sense_manager->Release();
	sense_manager->Close();
	destroyAllWindows();

	cout << "Performing left camera calibration" << endl;
	Size left_image_size(320, 240);
	Mat leftK;
	Mat leftD;
	int cal_flags_left = 0 + CALIB_FIX_K3;
	cv::calibrateCamera(left_object_points, left_image_points, left_image_size, leftK, leftD, noArray(), noArray(), cal_flags_left, TermCriteria(3, 20, 1e-6));
	cout << leftK << endl;
	cout << leftD << endl;

	cout << "Performing right camera calibration" << endl;
	Size right_image_size(320, 240);
	Mat rightK;
	Mat rightD;
	int cal_flags_right = 0 + CALIB_FIX_K3;
	cv::calibrateCamera(right_object_points, right_image_points, right_image_size, rightK, rightD, noArray(), noArray(), cal_flags_right, TermCriteria(3, 20, 1e-6));
	cout << rightK << endl;
	cout << rightD << endl;

	cout << "Performing stereo calibration" << endl;
	vector<vector<Point3f>> object_points;
	object_points.push_back(right_obj);
	object_points.push_back(left_obj);
	Mat E, F, R, T;
	int calib_flags = CALIB_FIX_INTRINSIC + 0 + CALIB_FIX_K3;
	cv::stereoCalibrate(object_points, left_image_points, right_image_points, leftK, leftD, rightK, rightD, left_image_size, R, T, E, F, calib_flags, TermCriteria(3, 20, 1e-6));
	
	cout << R << endl;
	cout << T << endl;
	cout << E << endl;
	cout << F << endl;

	ofstream stereo_file("ir_stereo_calibration.txt");
	if (stereo_file.is_open()){
		stereo_file << R.at<double>(0, 0) << endl;
		stereo_file << R.at<double>(0, 1) << endl;
		stereo_file << R.at<double>(0, 2) << endl;
		stereo_file << R.at<double>(1, 0) << endl;
		stereo_file << R.at<double>(1, 1) << endl;
		stereo_file << R.at<double>(1, 2) << endl;
		stereo_file << R.at<double>(2, 0) << endl;
		stereo_file << R.at<double>(2, 1) << endl;
		stereo_file << R.at<double>(2, 2) << endl;
		stereo_file << T.at<double>(0, 0) << endl;
		stereo_file << T.at<double>(0, 1) << endl;
		stereo_file << T.at<double>(0, 2) << endl;
		stereo_file << T.at<double>(0, 3) << endl;
		stereo_file.close();
	}
	else{
		cout << "Unable to open file" << endl;
	}


}