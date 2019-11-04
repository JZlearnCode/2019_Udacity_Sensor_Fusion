#ifndef UTIL_H
#define UTIL_H

#include <stdio.h>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
void loadImages(std::deque<DataFrame>* dataBuffer, std::string imgPrefix,
                int imgFillWidth, int imgStartIndex, int imgIndex,
                int dataBufferSize, std::string imgBasePath,
                std::string imgFileType);
void visMatches(std::string out_dir, std::string feature_detector_name,
                std::string feature_descriptor_name,
                std::deque<DataFrame>& dataBuffer);

#endif
