/* INCLUDES FOR THIS PROJECT */
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <sstream>
#include <vector>
#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {
  if (argc < 3) {
    // Tell the user how to run the program
    std::cerr << "Usage: " << argv[0] << "\n"
              << "Feature_detector_name:  SHITOMASI, HARRIS, FAST, BRISK, ORB, "
                 "AKAZE, SIFT \n"
              << "Feature_descriptor_name: BRIEF, ORB, FREAK, AKAZE, SIFT"
              << std::endl;
    return 1;
  }
  // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
  string detectorType = argv[1];
  // BRIEF, ORB, FREAK, AKAZE, SIFT
  string descriptorType = argv[2];
  std::cout << "Feature_detector_name: " << detectorType << std::endl;
  std::cout << "Feature_descriptor_name: " << descriptorType << std::endl;

  /* INIT VARIABLES AND DATA STRUCTURES */

  // data location
  string dataPath = "../";

  // camera
  string imgBasePath = dataPath + "images/";
  string imgPrefix =
      "KITTI/2011_09_26/image_00/data/000000";  // left camera, color
  string imgFileType = ".png";
  int imgStartIndex = 0;  // first file index to load (assumes Lidar and camera
                          // names have identical naming convention)
  int imgEndIndex = 9;    // last file index to load
  // no. of digits which make up the file index (e.g. img-0001.png)
  int imgFillWidth = 4;

  // misc
  int dataBufferSize = 2;  // no. of images which are held in memory (ring
                           // buffer) at the same time
  // use queue to have O(1) push to end and pop at the front
  deque<DataFrame> dataBuffer;
  bool bVis = false;  // visualize results

  /* MAIN LOOP OVER ALL IMAGES */

  for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex;
       imgIndex++) {
    /* LOAD IMAGE INTO BUFFER */

    // assemble filenames for current index
    ostringstream imgNumber;
    imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
    string imgFullFilename =
        imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

    // load image from file and convert to grayscale
    cv::Mat img, imgGray;
    img = cv::imread(imgFullFilename);
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // ring buffer of size dataBufferSize
    DataFrame frame;
    frame.cameraImg = imgGray;
    dataBuffer.push_back(frame);
    // remove old images
    if (dataBuffer.size() > dataBufferSize) {
      dataBuffer.pop_front();
    }

    cout << "#1 : RING BUFFER DONE" << endl;

    /* DETECT IMAGE KEYPOINTS */

    // extract 2D keypoints from current image
    vector<cv::KeyPoint> keypoints;
    // string-based selection of feature detection
    // HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    if (detectorType == "SHITOMASI") {
      detKeypointsShiTomasi(keypoints, imgGray);
    } else if (detectorType == "HARRIS") {
      detKeypointsHarris(keypoints, imgGray);
    } else {
      detKeypointsModern(keypoints, imgGray, detectorType);
    }
    //// EOF STUDENT ASSIGNMENT

    //// STUDENT ASSIGNMENT
    //// TASK MP.3 -> only keep keypoints on the preceding vehicle

    // only keep keypoints on the preceding vehicle
    bool bFocusOnVehicle = true;
    cv::Rect vehicleRect(535, 180, 180, 150);
    if (bFocusOnVehicle) {
      auto it = keypoints.begin();
      while (it != keypoints.end()) {
        if (vehicleRect.contains(it->pt)) {
          it++;
        } else {
          keypoints.erase(it);
        }
      }
    }  //// EOF STUDENT ASSIGNMENT

    // optional : limit number of keypoints (helpful for debugging and learning)
    bool bLimitKpts = false;
    if (bLimitKpts) {
      int maxKeypoints = 50;
      // there is no response info, so keep the first 50 as they are
      // sorted in descending quality order
      keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());

      cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
      cout << " NOTE: Keypoints have been limited!" << endl;
    }

    // push keypoints and descriptor for current frame to end of data buffer
    (dataBuffer.end() - 1)->keypoints = keypoints;
    cout << "#2 : DETECT KEYPOINTS done" << endl;

    /* EXTRACT KEYPOINT DESCRIPTORS */

    //// STUDENT ASSIGNMENT
    //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and
    /// enable string-based selection based on descriptorType / -> BRIEF, ORB,
    /// FREAK, AKAZE, SIFT

    cv::Mat descriptors;
    descKeypoints((dataBuffer.end() - 1)->keypoints,
                  (dataBuffer.end() - 1)->cameraImg, descriptors,
                  descriptorType);
    //// EOF STUDENT ASSIGNMENT

    // push descriptors for current frame to end of data buffer
    (dataBuffer.end() - 1)->descriptors = descriptors;

    cout << "#3 : EXTRACT DESCRIPTORS done" << endl;
    // wait until at least two images have been processed
    if (dataBuffer.size() > 1) {
      /* MATCH KEYPOINT DESCRIPTORS */

      vector<cv::DMatch> matches;
      string matcherType = "MAT_BF";         // MAT_BF, MAT_FLANN
      string descriptorType = "DES_BINARY";  // DES_BINARY, DES_HOG
      string selectorType = "SEL_NN";        // SEL_NN, SEL_KNN

      //// STUDENT ASSIGNMENT
      //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
      //// TASK MP.6 -> add KNN match selection and perform descriptor distance
      /// ratio filtering with t=0.8 in file matching2D.cpp

      matchDescriptors((dataBuffer.end() - 2)->keypoints,
                       (dataBuffer.end() - 1)->keypoints,
                       (dataBuffer.end() - 2)->descriptors,
                       (dataBuffer.end() - 1)->descriptors, matches,
                       descriptorType, matcherType, selectorType);

      //// EOF STUDENT ASSIGNMENT

      // store matches in current data frame
      (dataBuffer.end() - 1)->kptMatches = matches;

      cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

      // visualize matches between current and previous image
      bVis = true;
      if (bVis) {
        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
        cv::drawMatches((dataBuffer.end() - 2)->cameraImg,
                        (dataBuffer.end() - 2)->keypoints,
                        (dataBuffer.end() - 1)->cameraImg,
                        (dataBuffer.end() - 1)->keypoints, matches, matchImg,
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        vector<char>(),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        string windowName = "Matching keypoints between two camera images";
        cv::namedWindow(windowName, 7);
        cv::imshow(windowName, matchImg);
        cout << "Press key to continue to next image" << endl;
        cv::waitKey(0);  // wait for key to be pressed
      }
      bVis = false;
    }

  }  // eof loop over all images

  return 0;
}
