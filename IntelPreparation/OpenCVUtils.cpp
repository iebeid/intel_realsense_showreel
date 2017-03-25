#include "OpenCVUtils.h"
#include "RealSenseUtils.h"

//static void
//StereoCalib(const char* imageList, int nx, int ny, int useUncalibrated)
//{
//	int displayCorners = 0;
//	int showUndistorted = 1;
//	bool isVerticalStereo = false;//OpenCV can handle left-right
//	//or up-down camera arrangements
//	const int maxScale = 1;
//	const float squareSize = 1.f; //Set this to your actual square size
//	FILE* f = fopen(imageList, "rt");
//	int i, j, lr, nframes, n = nx*ny, N = 0;
//	vector<string> imageNames[2];
//	vector<CvPoint3D32f> objectPoints;
//	vector<CvPoint2D32f> points[2];
//	vector<int> npoints;
//	vector<uchar> active[2];
//	vector<CvPoint2D32f> temp(n);
//	CvSize imageSize = { 0, 0 };
//	// ARRAY AND VECTOR STORAGE:
//	double M1[3][3], M2[3][3], D1[5], D2[5];
//	double R[3][3], T[3], E[3][3], F[3][3];
//	CvMat _M1 = cvMat(3, 3, CV_64F, M1);
//	CvMat _M2 = cvMat(3, 3, CV_64F, M2);
//	CvMat _D1 = cvMat(1, 5, CV_64F, D1);
//	CvMat _D2 = cvMat(1, 5, CV_64F, D2);
//	CvMat _R = cvMat(3, 3, CV_64F, R);
//	CvMat _T = cvMat(3, 1, CV_64F, T);
//	CvMat _E = cvMat(3, 3, CV_64F, E);
//	CvMat _F = cvMat(3, 3, CV_64F, F);
//	if (displayCorners)
//		cvNamedWindow("corners", 1);
//	// READ IN THE LIST OF CHESSBOARDS:
//	if (!f)
//	{
//		fprintf(stderr, "can not open file %s\n", imageList);
//		return;
//	}
//	for (i = 0;; i++)
//	{
//		char buf[1024];
//		int count = 0, result = 0;
//		lr = i % 2;
//		vector<CvPoint2D32f>& pts = points[lr];
//		if (!fgets(buf, sizeof(buf) - 3, f))
//			break;
//		size_t len = strlen(buf);
//		while (len > 0 && isspace(buf[len - 1]))
//			buf[--len] = '\0';
//		if (buf[0] == '#')
//			continue;
//		IplImage* img = cvLoadImage(buf, 0);
//		if (!img)
//			break;
//		imageSize = cvGetSize(img);
//		imageNames[lr].push_back(buf);
//		//FIND CHESSBOARDS AND CORNERS THEREIN:
//		for (int s = 1; s <= maxScale; s++)
//		{
//			IplImage* timg = img;
//			if (s > 1)
//			{
//				timg = cvCreateImage(cvSize(img->width*s, img->height*s),
//					img->depth, img->nChannels);
//				cvResize(img, timg, CV_INTER_CUBIC);
//			}
//			result = cvFindChessboardCorners(timg, cvSize(nx, ny),
//				&temp[0], &count,
//				CV_CALIB_CB_ADAPTIVE_THRESH |
//				CV_CALIB_CB_NORMALIZE_IMAGE);
//			if (timg != img)
//				cvReleaseImage(&timg);
//			if (result || s == maxScale)
//				for (j = 0; j < count; j++)
//				{
//					temp[j].x /= s;
//					temp[j].y /= s;
//				}
//			if (result)
//				break;
//		}
//		if (displayCorners)
//		{
//			printf("%s\n", buf);
//			IplImage* cimg = cvCreateImage(imageSize, 8, 3);
//			cvCvtColor(img, cimg, CV_GRAY2BGR);
//			cvDrawChessboardCorners(cimg, cvSize(nx, ny), &temp[0],
//				count, result);
//			cvShowImage("corners", cimg);
//			cvReleaseImage(&cimg);
//			if (cvWaitKey(0) == 27) //Allow ESC to quit
//				exit(-1);
//		}
//		else
//			putchar('.');
//		N = pts.size();
//		pts.resize(N + n, cvPoint2D32f(0, 0));
//		active[lr].push_back((uchar)result);
//		//assert( result != 0 );
//		if (result)
//		{
//			//Calibration will suffer without subpixel interpolation
//			cvFindCornerSubPix(img, &temp[0], count,
//				cvSize(11, 11), cvSize(-1, -1),
//				cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,
//				30, 0.01));
//			copy(temp.begin(), temp.end(), pts.begin() + N);
//		}
//		cvReleaseImage(&img);
//	}
//	fclose(f);
//	printf("\n");
//	// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
//	nframes = active[0].size();//Number of good chessboads found
//	objectPoints.resize(nframes*n);
//	for (i = 0; i < ny; i++)
//		for (j = 0; j < nx; j++)
//			objectPoints[i*nx + j] =
//			cvPoint3D32f(i*squareSize, j*squareSize, 0);
//	for (i = 1; i < nframes; i++)
//		copy(objectPoints.begin(), objectPoints.begin() + n,
//		objectPoints.begin() + i*n);
//	npoints.resize(nframes, n);
//	N = nframes*n;
//	CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0]);
//	CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0]);
//	CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0]);
//	CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0]);
//	cvSetIdentity(&_M1);
//	cvSetIdentity(&_M2);
//	cvZero(&_D1);
//	cvZero(&_D2);
//	// CALIBRATE THE STEREO CAMERAS
//	printf("Running stereo calibration ...");
//	fflush(stdout);
//	cvStereoCalibrate(&_objectPoints, &_imagePoints1,
//		&_imagePoints2, &_npoints,
//		&_M1, &_D1, &_M2, &_D2,
//		imageSize, &_R, &_T, &_E, &_F,
//		cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5), CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_SAME_FOCAL_LENGTH);
//	printf(" done\n");
//	// CALIBRATION QUALITY CHECK
//	// because the output fundamental matrix implicitly
//	// includes all the output information,
//	// we can check the quality of calibration using the
//	// epipolar geometry constraint: m2^t*F*m1=0
//	vector<CvPoint3D32f> lines[2];
//	points[0].resize(N);
//	points[1].resize(N);
//	_imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0]);
//	_imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0]);
//	lines[0].resize(N);
//	lines[1].resize(N);
//	CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
//	CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
//	//Always work in undistorted space
//	cvUndistortPoints(&_imagePoints1, &_imagePoints1,
//		&_M1, &_D1, 0, &_M1);
//	cvUndistortPoints(&_imagePoints2, &_imagePoints2,
//		&_M2, &_D2, 0, &_M2);
//	cvComputeCorrespondEpilines(&_imagePoints1, 1, &_F, &_L1);
//	cvComputeCorrespondEpilines(&_imagePoints2, 2, &_F, &_L2);
//	double avgErr = 0;
//	for (i = 0; i < N; i++)
//	{
//		double err = fabs(points[0][i].x*lines[1][i].x +
//			points[0][i].y*lines[1][i].y + lines[1][i].z)
//			+ fabs(points[1][i].x*lines[0][i].x +
//			points[1][i].y*lines[0][i].y + lines[0][i].z);
//		avgErr += err;
//	}
//	printf("avg err = %g\n", avgErr / (nframes*n));
//	//COMPUTE AND DISPLAY RECTIFICATION
//	if (showUndistorted)
//	{
//		CvMat* mx1 = cvCreateMat(imageSize.height,
//			imageSize.width, CV_32F);
//		CvMat* my1 = cvCreateMat(imageSize.height,
//			imageSize.width, CV_32F);
//		CvMat* mx2 = cvCreateMat(imageSize.height,
//			imageSize.width, CV_32F);
//		CvMat* my2 = cvCreateMat(imageSize.height,
//			imageSize.width, CV_32F);
//		CvMat* img1r = cvCreateMat(imageSize.height,
//			imageSize.width, CV_8U);
//		CvMat* img2r = cvCreateMat(imageSize.height,
//			imageSize.width, CV_8U);
//		CvMat* disp = cvCreateMat(imageSize.height,
//			imageSize.width, CV_16S);
//		CvMat* vdisp = cvCreateMat(imageSize.height,
//			imageSize.width, CV_8U);
//		CvMat* pair;
//		double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
//		CvMat _R1 = cvMat(3, 3, CV_64F, R1);
//		CvMat _R2 = cvMat(3, 3, CV_64F, R2);
//		// IF BY CALIBRATED (BOUGUET'S METHOD)
//		if (useUncalibrated == 0)
//		{
//			CvMat _P1 = cvMat(3, 4, CV_64F, P1);
//			CvMat _P2 = cvMat(3, 4, CV_64F, P2);
//			cvStereoRectify(&_M1, &_M2, &_D1, &_D2, imageSize,
//				&_R, &_T,
//				&_R1, &_R2, &_P1, &_P2, 0,
//				0/*CV_CALIB_ZERO_DISPARITY*/);
//			isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
//			//Precompute maps for cvRemap()
//			cvInitUndistortRectifyMap(&_M1, &_D1, &_R1, &_P1, mx1, my1);
//			cvInitUndistortRectifyMap(&_M2, &_D2, &_R2, &_P2, mx2, my2);
//		}
//		//OR ELSE HARTLEY'S METHOD
//		else if (useUncalibrated == 1 || useUncalibrated == 2)
//			// use intrinsic parameters of each camera, but
//			// compute the rectification transformation directly
//			// from the fundamental matrix
//		{
//			double H1[3][3], H2[3][3], iM[3][3];
//			CvMat _H1 = cvMat(3, 3, CV_64F, H1);
//			CvMat _H2 = cvMat(3, 3, CV_64F, H2);
//			CvMat _iM = cvMat(3, 3, CV_64F, iM);
//			//Just to show you could have independently used F
//			if (useUncalibrated == 2)
//				cvFindFundamentalMat(&_imagePoints1,
//				&_imagePoints2, &_F);
//			cvStereoRectifyUncalibrated(&_imagePoints1,
//				&_imagePoints2, &_F,
//				imageSize,
//				&_H1, &_H2, 3);
//			cvInvert(&_M1, &_iM);
//			cvMatMul(&_H1, &_M1, &_R1);
//			cvMatMul(&_iM, &_R1, &_R1);
//			cvInvert(&_M2, &_iM);
//			cvMatMul(&_H2, &_M2, &_R2);
//			cvMatMul(&_iM, &_R2, &_R2);
//			//Precompute map for cvRemap()
//			cvInitUndistortRectifyMap(&_M1, &_D1, &_R1, &_M1, mx1, my1);
//			cvInitUndistortRectifyMap(&_M2, &_D1, &_R2, &_M2, mx2, my2);
//		}
//		else
//			assert(0);
//		cvNamedWindow("rectified", 1);
//		// RECTIFY THE IMAGES AND FIND DISPARITY MAPS
//		if (!isVerticalStereo)
//			pair = cvCreateMat(imageSize.height, imageSize.width * 2,
//			CV_8UC3);
//		else
//			pair = cvCreateMat(imageSize.height * 2, imageSize.width,
//			CV_8UC3);
//		//Setup for finding stereo correspondences
//		CvStereoBMState *BMState = cvCreateStereoBMState();
//		assert(BMState != 0);
//		BMState->preFilterSize = 41;
//		BMState->preFilterCap = 31;
//		BMState->SADWindowSize = 41;
//		BMState->minDisparity = -64;
//		BMState->numberOfDisparities = 128;
//		BMState->textureThreshold = 10;
//		BMState->uniquenessRatio = 15;
//		for (i = 0; i < nframes; i++)
//		{
//			IplImage* img1 = cvLoadImage(imageNames[0][i].c_str(), 0);
//			IplImage* img2 = cvLoadImage(imageNames[1][i].c_str(), 0);
//			if (img1 && img2)
//			{
//				CvMat part;
//				cvRemap(img1, img1r, mx1, my1);
//				cvRemap(img2, img2r, mx2, my2);
//				if (!isVerticalStereo || useUncalibrated != 0)
//				{
//					// When the stereo camera is oriented vertically,
//					// useUncalibrated==0 does not transpose the
//					// image, so the epipolar lines in the rectified
//					// images are vertical. Stereo correspondence
//					// function does not support such a case.
//					cvFindStereoCorrespondenceBM(img1r, img2r, disp,
//						BMState);
//					cvNormalize(disp, vdisp, 0, 256, CV_MINMAX);
//					cvNamedWindow("disparity");
//					cvShowImage("disparity", vdisp);
//				}
//				if (!isVerticalStereo)
//				{
//					cvGetCols(pair, &part, 0, imageSize.width);
//					cvCvtColor(img1r, &part, CV_GRAY2BGR);
//					cvGetCols(pair, &part, imageSize.width,
//						imageSize.width * 2);
//					cvCvtColor(img2r, &part, CV_GRAY2BGR);
//					for (j = 0; j < imageSize.height; j += 16)
//						cvLine(pair, cvPoint(0, j),
//						cvPoint(imageSize.width * 2, j),
//						CV_RGB(0, 255, 0));
//				}
//				else
//				{
//					cvGetRows(pair, &part, 0, imageSize.height);
//					cvCvtColor(img1r, &part, CV_GRAY2BGR);
//					cvGetRows(pair, &part, imageSize.height,
//						imageSize.height * 2);
//					cvCvtColor(img2r, &part, CV_GRAY2BGR);
//					for (j = 0; j < imageSize.width; j += 16)
//						cvLine(pair, cvPoint(j, 0),
//						cvPoint(j, imageSize.height * 2),
//						CV_RGB(0, 255, 0));
//				}
//				cvShowImage("rectified", pair);
//				if (cvWaitKey() == 27)
//					break;
//			}
//			cvReleaseImage(&img1);
//			cvReleaseImage(&img2);
//		}
//		cvReleaseStereoBMState(&BMState);
//		cvReleaseMat(&mx1);
//		cvReleaseMat(&my1);
//		cvReleaseMat(&mx2);
//		cvReleaseMat(&my2);
//		cvReleaseMat(&img1r);
//		cvReleaseMat(&img2r);
//		cvReleaseMat(&disp);
//	}
//}



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
	//int cal_flags_left = CALIB_FIX_ASPECT_RATIO + CALIB_ZERO_TANGENT_DIST + CALIB_FIX_PRINCIPAL_POINT + CALIB_FIX_K1 + CALIB_FIX_K2 + CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5;
	//cv::calibrateCamera(object_points, left_image_points, left_image_size, leftK, leftD, noArray(), noArray(), cal_flags_left, TermCriteria(3, 20, 1e-6));
	vector<Mat> rvecs_left; 
	vector<Mat> tvecs_left;
	//double left_rms = calibrateCamera(object_points, left_image_points, left_image_size, leftK, leftD, rvecs_left, tvecs_left, CV_CALIB_USE_INTRINSIC_GUESS);
	leftK = cv::initCameraMatrix2D(object_points, left_image_points, left_image_size,0);
	//cv::solvePnP(Mat(object_points), Mat(left_image_points), leftK, leftD,noArray(),noArray());
	//Mat optimal_left_K = cv::getOptimalNewCameraMatrix(leftK, leftD, left_image_size, 1, left_image_size);
	cout << leftK << endl;
	cout << leftD << endl;
	//vector<vector<Point2f>> undistorted_left_image_points;
	//cv::Mat left_points_input(1, (int)left_image_points.size(), CV_32FC2);
	//cv::Mat undistorted_left_points_input();
	//cv::undistortPoints(left_image_points, undistorted_left_image_points, optimal_left_K, leftD, noArray(), noArray());

	cout << "Performing right camera calibration" << endl;
	Size right_image_size(320, 240);
	Mat rightK;
	Mat rightD;
	//int cal_flags_right = cv::CALIB_RATIONAL_MODEL + cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5 + cv::CALIB_FIX_K6;
	vector<Mat> rvecs_right;
	vector<Mat> tvecs_right;
	//double right_rms = calibrateCamera(object_points, right_image_points, right_image_size, rightK, rightD, rvecs_right, tvecs_right, CV_CALIB_USE_INTRINSIC_GUESS);
	rightK = cv::initCameraMatrix2D(object_points, right_image_points, right_image_size,0);
	//cv::calibrateCamera(object_points, right_image_points, right_image_size, rightK, rightD, noArray(), noArray(), cal_flags_left, TermCriteria(3, 20, 1e-6));
	//cv::solvePnP(object_points, right_image_points, rightK, rightD, noArray(), noArray());
	//Mat optimal_right_K = cv::getOptimalNewCameraMatrix(rightK, rightD, right_image_size, 1, right_image_size);
	cout << rightK << endl;
	cout << rightD << endl;
	//vector<vector<Point2f>> undistorted_right_image_points;
	//cv::undistortPoints(right_image_points, undistorted_right_image_points, optimal_right_K, rightD, noArray(), noArray());

	cout << "Performing stereo calibration" << endl;
	//Mat E, F, R, T;
	//int calib_flags = CALIB_FIX_INTRINSIC + 0 + CALIB_FIX_K3;
	//int calib_flags = cv::CALIB_RATIONAL_MODEL + cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5 + cv::CALIB_FIX_K6;
	//cv::stereoCalibrate(object_points, left_image_points, right_image_points, optimal_left_K, leftD, optimal_right_K, rightD, left_image_size, R, T, E, F, calib_flags, TermCriteria(3, 20, 1e-6));

	Mat R, T, E, F;

	double rms = stereoCalibrate(object_points, left_image_points, right_image_points,
		leftK, leftD,
		rightK, rightD,
		left_image_size, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO +
		CALIB_ZERO_TANGENT_DIST +
		CALIB_USE_INTRINSIC_GUESS +
		CALIB_SAME_FOCAL_LENGTH +
		CALIB_RATIONAL_MODEL +
		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	cout << "done with RMS error=" << rms << endl;



	cout << R << endl;
	cout << T << endl;
	cout << E << endl;
	cout << F << endl;

	//cv::Mat R1, R2, P1, P2, Q;

	//cv::stereoRectify(optimal_left_K, leftD, optimal_right_K, rightD, left_image_size, R, T, R1, R2, P1, P2, Q, CV_CALIB_ZERO_DISPARITY, 0);

	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	stereoRectify(leftK, leftD,
		rightK, rightD,
		left_image_size, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, left_image_size, &validRoi[0], &validRoi[1]);

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