
#ifndef matching2D_hpp
#define matching2D_hpp

#include <stdio.h>
#include <iostream>
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

#include "dataStructures.hpp"


bool compareKeypointsHarris(cv::KeyPoint p1, cv::KeyPoint p2);
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img);
void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img);
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                        std::string detectorType);
void detectKeyPoints(std::deque<DataFrame> *dataBuffer,
                     std::string detectorType);
void descKeypoints(std::deque<DataFrame> *dataBuffer,
                   std::string descriptorType);
cv::Ptr<cv::DescriptorMatcher> createMatcher(std::string descriptorType,
                                             std::string matcherType);
void matchDescriptors(std::deque<DataFrame> *dataBuffer,
                      std::string descriptorType, std::string matcherType,
                      std::string selectorType);
#endif /* matching2D_hpp */
