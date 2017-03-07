#ifndef REAL_SENSE_UTILS
#define REAL_SENSE_UTILS 1

#include <pxcsensemanager.h>
#include <iostream>

PXCCapture::Device* real_sense_info(PXCCaptureManager *pCaptureManager);

PXCSenseManager* init_real_sense(int width, int height, PXCCapture::Device** device, PXCSession** session);

PXCImage* map_color_to_depth(PXCImage* depth, PXCImage* color, PXCSession* pSession);

PXCImage* filter_depth(PXCImage * depth_image, PXCSession * pSession, int low_threshold, int high_threshold, short * pSysMemSrc, short * pSysMemDst);

#endif