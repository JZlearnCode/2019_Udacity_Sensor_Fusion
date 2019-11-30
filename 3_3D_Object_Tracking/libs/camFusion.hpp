
#ifndef camFusion_hpp
#define camFusion_hpp

#include <stdio.h>
#include <vector>
#include <deque>
#include <opencv2/core.hpp>
#include "dataStructures.h"
#include "lidarData.hpp"
#include "inputOutputUtil.h"

void detectLidar(std::string imgBasePath, std::string lidarPrefix, std::string imgNumber, std::string lidarFileType,
                 cv::Mat P_rect_00, cv::Mat R_rect_00, cv::Mat RT, std::deque<DataFrame>* dataBuffer);
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT);
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches);
void matchBoundingBoxes(std::deque<DataFrame>* dataBuffer);

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait=true);

void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg=nullptr);
bool lidarAscendingX(LidarPoint a, LidarPoint b);
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC); 

void calculateTTCCombined(std::deque<DataFrame>* dataBuffer, double sensorFrameRate,  
                  cv::Mat P_rect_00, cv::Mat R_rect_00, cv::Mat RT, 
                  std::vector<float>* distDifference, bool bVis);         
#endif /* camFusion_hpp */
