#ifndef loadData_h
#define loadData_h
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <deque>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"


// set parameters for calibration
// do not need to pass by reference since 
// "Mat is a structure that keeps matrix/image characteristics (rows and columns number, data type etc) and a pointer to data"
void loadCalibrationParams(cv::Mat P_rect_00, cv::Mat R_rect_00, cv::Mat RT);

// Load images into buffer
std::string loadImages(std::deque<DataFrame>* dataBuffer, std::string imgPrefix,
                int imgFillWidth, int imgStartIndex, int imgIndex,
                int dataBufferSize, std::string imgBasePath, std::string imgFileType);

#endif /* loadData_h */
