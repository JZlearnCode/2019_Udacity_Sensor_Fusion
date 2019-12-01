
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

void detectLidar(string imgBasePath, string lidarPrefix, string imgNumber, string lidarFileType,
                 cv::Mat P_rect_00, cv::Mat R_rect_00, cv::Mat RT,
                 std::deque<DataFrame>* dataBuffer, bool bVis) {
    // load 3D Lidar points from file
    string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber + lidarFileType;
    std::vector<LidarPoint> lidarPoints;
    loadLidarFromFile(lidarPoints, lidarFullFilename);
    if (bVis){
        std::cout<<"raw lidar data"<<std::endl;
        showLidarTopview(lidarPoints, cv::Size(4.0, 20.0), cv::Size(2000, 2000), bVis);
    }
    // remove Lidar points based on distance properties
    float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
    cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
    (dataBuffer->end() - 1)->lidarPoints = lidarPoints;
    if (bVis){
        std::cout<<"raw lidar data"<<std::endl;
        showLidarTopview(lidarPoints, cv::Size(4.0, 20.0), cv::Size(2000, 2000), bVis);
    }

    /* CLUSTER LIDAR POINT CLOUD */
    // associate Lidar points with camera-based ROI
    float shrinkFactor = 0.2; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
    clusterLidarWithROI((dataBuffer->end()-1)->boundingBoxes, 
                        (dataBuffer->end() - 1)->lidarPoints, 
                        shrinkFactor, P_rect_00, R_rect_00, RT);
    if (bVis){
        std::cout<<"remove lidar points outside bbox"<<std::endl;
        showLidarTopview(lidarPoints, cv::Size(4.0, 20.0), cv::Size(2000, 2000), bVis);
    }
}


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {   
                //enclsingBoxes stores iterators for all bbox that contains current Lidar point
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // only keep Lidar point enclosed by a single bbox 
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    //keep points inside the bounding box
    std::vector<cv::DMatch> candidateMatches;
    float sumDist; 
    for(cv::DMatch& match : kptMatches) {
        cv::Point2f curKeyPoint= kptsCurr[match.trainIdx].pt;
        if(boundingBox.roi.contains(curKeyPoint)) {
            candidateMatches.push_back(match);

            cv::Point2f prevKeyPoint= kptsPrev[match.queryIdx].pt;
            float dist = cv::norm(curKeyPoint - prevKeyPoint);
            sumDist+=dist;
        }
    }
    // matches with distance larger than the median distance is treated as outlier
    float meanDist = sumDist / candidateMatches.size();
    const double kRatio = 1.5;
    for(size_t i = 0; i<candidateMatches.size(); i++) {
        cv::Point2f curKeyPoint = kptsCurr[candidateMatches[i].trainIdx].pt;
        cv::Point2f prevKeyPoint = kptsPrev[candidateMatches[i].queryIdx].pt;
        float dist = cv::norm(curKeyPoint - prevKeyPoint);
        if (dist <= meanDist*kRatio) {
            // save inlier
            boundingBox.kptMatches.push_back(candidateMatches[i]);
        }
    }

}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    //compute all distance ratios between all matched keypoints
    vector<double> distRatios; 
    double minDist = 10.0; //minimum required distance 
    //use an outer loop and inner loop to compute distance between
    //each keypoint to all the other keypoints 
    for (size_t i = 0; i < kptMatches.size(); i++) {
        cv::Point2f outerCurrKpt = kptsCurr[kptMatches[i].trainIdx].pt;
        cv::Point2f outerPrevKpt = kptsPrev[kptMatches[i].queryIdx].pt;

        for (size_t j = i+1; j<kptMatches.size(); j++) {
            cv::Point2f innerCurrKpt = kptsCurr[kptMatches[j].trainIdx].pt;
            cv::Point2f innerPrevKpt = kptsPrev[kptMatches[j].queryIdx].pt;
            //compute distance ratios
            double distCurrKpts= cv::norm(outerCurrKpt - innerCurrKpt);
            double distPrevKpts = cv::norm(outerPrevKpt - innerPrevKpt);
            //avoid division by zero 
            if (distPrevKpts > std::numeric_limits<double>::epsilon() && distCurrKpts > minDist) {
                double distRatio = distCurrKpts / distPrevKpts;
                distRatios.push_back(distRatio);
            }
        } 
     }

     // use mean distance to avoid wrong result introduced from outliers 
     if (distRatios.size() == 0) {
         TTC = NAN;
         return; 
     }

     double sumDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0);
     double meanDistRatio = sumDistRatio / distRatios.size();

     double dT = 1.0/frameRate;
     TTC = -dT / (1 - meanDistRatio);
}

bool lidarAscendingX(LidarPoint a, LidarPoint b) {
    return a.x < b.x;; 
}
 

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    sort(lidarPointsPrev.begin(), lidarPointsPrev.end(), lidarAscendingX);
    int prev_median_idx = lidarPointsPrev.size()/2;
    double dist_prev = lidarPointsPrev[prev_median_idx].x; 

    sort(lidarPointsCurr.begin(), lidarPointsCurr.end(), lidarAscendingX);
    int cur_median_idx = lidarPointsCurr.size()/2;
    double dist_cur = lidarPointsCurr[cur_median_idx].x; 
    // constant velocity model
    TTC = dist_cur * (1.0 / frameRate) / (dist_prev - dist_cur);
}

void matchBoundingBoxes(deque<DataFrame>* dataBuffer)
{
    map<int, int> bbBestMatches;
    std::vector<cv::DMatch> matches = (dataBuffer->end() - 1)->kptMatches;
    DataFrame& prevFrame = *(dataBuffer->end()-2);
    DataFrame& currFrame = *(dataBuffer->end()-1);
 
    for(auto &prevFrameBox : prevFrame.boundingBoxes) {
        //key: box id for a bounding box in current frame
        //value: number of keypoint matches between the bbox in current frame and the bbox in previous frame
        std::unordered_map<int, int> matchesPerBox; 
        for(auto& currFrameBox : currFrame.boundingBoxes) {
            // count number of matches for each of the current bbox 
            for(auto match : matches) {
                bool insidePrevBox = prevFrameBox.roi.contains(prevFrame.keypoints[match.queryIdx].pt);
                bool insideCurBox = currFrameBox.roi.contains(currFrame.keypoints[match.trainIdx].pt);
                if(insidePrevBox && insideCurBox) {
                    matchesPerBox[currFrameBox.boxID]++;
                }
            }
        }
        //get index for bounding box in current frame with maximum number of matches with a bounding box in the previous frame 
        int max_matches = 0;
        int max_match_cur_boxid = 0;  
        for(auto& pair : matchesPerBox) {
            if (pair.second > max_matches) {
                max_match_cur_boxid = pair.first;
                max_matches = pair.second; 
            }
        }
        bbBestMatches[prevFrameBox.boxID] = max_match_cur_boxid; 
    }
    // store matches in current data frame
    (dataBuffer->end()-1)->bbMatches = bbBestMatches;
}


void calculateTTCCombined(std::deque<DataFrame>* dataBuffer, double sensorFrameRate,  
                  cv::Mat P_rect_00, cv::Mat R_rect_00, cv::Mat RT, 
                  vector<float>* distDifference, bool bVis) {
    // loop over all BB match pairs
    for (auto it1 = (dataBuffer->end() - 1)->bbMatches.begin(); it1 != (dataBuffer->end() - 1)->bbMatches.end(); ++it1)
    {
        // find bounding boxes associates with current match
        BoundingBox *prevBB, *currBB;
        for (auto it2 = (dataBuffer->end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer->end() - 1)->boundingBoxes.end(); ++it2)
        {
            if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
            {
                currBB = &(*it2);
            }
        }

        for (auto it2 = (dataBuffer->end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer->end() - 2)->boundingBoxes.end(); ++it2)
        {
            if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
            {
                prevBB = &(*it2);
            }
        }

        // compute TTC for current match
        if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) // only compute TTC if we have Lidar points
        {
            double ttcLidar; 
            computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
            std::cout<<"ttcLidar"<<ttcLidar<<std::endl;
            double ttcCamera;
            clusterKptMatchesWithROI(*currBB, (dataBuffer->end() - 2)->keypoints, (dataBuffer->end() - 1)->keypoints, (dataBuffer->end() - 1)->kptMatches);                    
            computeTTCCamera((dataBuffer->end() - 2)->keypoints, (dataBuffer->end() - 1)->keypoints, currBB->kptMatches, sensorFrameRate, ttcCamera);
            std::cout<<"ttcCamera"<<ttcCamera<<std::endl;

            distDifference->push_back(abs(ttcLidar - ttcCamera));

            visResult(*dataBuffer, currBB, bVis, P_rect_00, R_rect_00, RT,
                        ttcLidar, ttcCamera); 
        } // eof TTC computation
    } // eof loop over all BB matches 
}