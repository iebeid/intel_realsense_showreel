
#include "RealSenseUtils.h"

using namespace std;

PXCSenseManager* init_real_sense(int width, int height, PXCCapture::Device** device, PXCSession** session){
	cout << "--------------------------" << endl;
	//Camera initialization
	PXCSession* current_session = PXCSession::CreateInstance();
	PXCSession::ImplVersion ver = current_session->QueryVersion();
	cout << "SDK Version: " << ver.major << "." << ver.minor << endl;
	current_session->SetCoordinateSystem(PXCSession::CoordinateSystem::COORDINATE_SYSTEM_REAR_OPENCV);
	PXCSenseManager *sense_manager = current_session->CreateSenseManager();
	
	sense_manager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, width, height, 30);
	sense_manager->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, width, height, 30);
	sense_manager->EnableStream(PXCCapture::STREAM_TYPE_LEFT, width, height, 30);
	sense_manager->EnableStream(PXCCapture::STREAM_TYPE_RIGHT, width, height, 30);
	
	sense_manager->Init();
	PXCCaptureManager *pCaptureManager = sense_manager->QueryCaptureManager();
	//PXCCapture::Device* current_device = real_sense_info(capture_manager);
	//PXCCapture::Device* current_device;

	//Info
	PXCCapture::Device *current_device = pCaptureManager->QueryDevice();
	PXCCapture::DeviceInfo device_info = {};
	current_device->QueryDeviceInfo(&device_info);
	wprintf(device_info.name);
	cout << endl;
	cout << "Firmware: " << device_info.firmware[0] << "." << device_info.firmware[1] << "." << device_info.firmware[2] << "." << device_info.firmware[3] << endl;
	PXCPointF32 fov = current_device->QueryDepthFieldOfView();
	cout << "Depth Horizontal Field Of View: " << fov.x << endl;
	cout << "Depth Vertical Field Of View: " << fov.y << endl;
	PXCSizeI32 csize = pCaptureManager->QueryImageSize(PXCCapture::STREAM_TYPE_COLOR);
	cout << "Color Resolution: " << csize.width << " * " << csize.height << endl;
	PXCSizeI32 dsize = pCaptureManager->QueryImageSize(PXCCapture::STREAM_TYPE_DEPTH);
	cout << "Depth Resolution: " << dsize.width << " * " << dsize.height << endl;
	PXCSizeI32 leftsize = pCaptureManager->QueryImageSize(PXCCapture::STREAM_TYPE_LEFT);
	cout << "Left Resolution: " << leftsize.width << " * " << leftsize.height << endl;
	PXCSizeI32 rightsize = pCaptureManager->QueryImageSize(PXCCapture::STREAM_TYPE_RIGHT);
	cout << "Right Resolution: " << rightsize.width << " * " << rightsize.height << endl;

	////Camera calibration
	//cout << "Calibrating" << endl;
	//current_device->SetDepthConfidenceThreshold(6);
	//current_device->SetIVCAMFilterOption(5);
	//current_device->SetIVCAMLaserPower(16);
	//current_device->SetIVCAMMotionRangeTradeOff(16);
	//current_device->SetIVCAMAccuracy(current_device->IVCAM_ACCURACY_MEDIAN);
	//cout << "Depth Setting - OK - Calibrated" << endl;


	cout << "Camera Initialized" << endl;
	*device = current_device;
	*session = current_session;
	cout << "--------------------------" << endl;
	return sense_manager;
}

PXCImage * map_color_to_depth(PXCImage * depth, PXCImage * color, PXCSession *pSession){
	PXCImage *color_image = color;
	PXCImage::ImageInfo color_info = color_image->QueryInfo();
	PXCImage::ImageData color_data;
	color_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &color_data);
	unsigned char * cpixels = (unsigned char *)color_data.planes[0];
	int cpitch = color_data.pitches[0];
	color_image->ReleaseAccess(&color_data);
	PXCImage *depth_image = depth;
	PXCImage::ImageInfo depth_info = depth_image->QueryInfo();
	PXCImage::ImageData depth_data;
	depth_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &depth_data);
	short *dpixels = (short*)depth_data.planes[0];
	int dpitch = depth_data.pitches[0] / sizeof(short);
	depth_image->ReleaseAccess(&depth_data);
	pxcBYTE * mapped_colored_image_manual = (pxcBYTE *)malloc(depth_info.height*depth_info.width*sizeof(pxcBYTE) * 4);
	int h = 0;
	for (int y = 0; y < depth_info.height; y++){
		for (int x = 0; x < depth_info.width; x++){
			short depth_value = dpixels[(y * dpitch) + x];
			pxcBYTE blue, green, red, alpha;
			if (depth_value != 0){
				blue = (pxcBYTE)(cpixels + y*cpitch)[4 * x + 0];
				green = (pxcBYTE)(cpixels + y*cpitch)[4 * x + 1];
				red = (pxcBYTE)(cpixels + y*cpitch)[4 * x + 2];
				alpha = (pxcBYTE)(cpixels + y*cpitch)[4 * x + 3];
			}
			else{
				blue = 0;
				green = 0;
				red = 0;
				alpha = 0;
			}
			mapped_colored_image_manual[h] = blue;
			mapped_colored_image_manual[h + 1] = green;
			mapped_colored_image_manual[h + 2] = red;
			mapped_colored_image_manual[h + 3] = alpha;
			h = h + 4;
		}
	}
	PXCImage * mapped_image;
	PXCImage::ImageInfo info = {};
	info.format = PXCImage::PIXEL_FORMAT_RGB32;
	info.width = depth_info.width;
	info.height = depth_info.height;
	mapped_image = pSession->CreateImage(&info);
	PXCImage::ImageData data;
	mapped_image->AcquireAccess(PXCImage::ACCESS_WRITE, &data);
	memcpy(data.planes[0], mapped_colored_image_manual, depth_info.height*depth_info.width*sizeof(pxcBYTE) * 4);
	free(mapped_colored_image_manual);
	mapped_image->ReleaseAccess(&data);
	return mapped_image;
}