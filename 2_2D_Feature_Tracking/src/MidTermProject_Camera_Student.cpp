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
#include <unordered_set>
#include <vector>
#include "dataStructures.h"
#include "matching2D.hpp"
#include "util.h"
using namespace std;

// test performance of feature detector and feature descritor combinations
void featureTrackingPipelinle(string imgBasePath, string out_dir,
                              string feature_detector_name,
                              string feature_descriptor_name) {
  /* INIT VARIABLES AND DATA STRUCTURES */
  string imgPrefix = "KITTI/2011_09_26/image_00/data/000000";
  string imgFileType = ".png";
  // first and last file index to load
  int imgStartIndex = 0;
  int imgEndIndex = 9;
  // no. of digits which make up the file index (e.g. img-0001.png)
  int imgFillWidth = 4;
  // no. of images which are held in memory (ring buffer) at the same time
  int dataBufferSize = 2;
  // use queue to have O(1) push to end and pop at the front
  deque<DataFrame> dataBuffer;
  unordered_set<string> binary_descriptors = {"BRIEF", "ORB", "FREAK", "AKAZE"};
  bool bVis = true;
  /* MAIN LOOP OVER ALL IMAGES */
  for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex;
       imgIndex++) {
    loadImages(&dataBuffer, imgPrefix, imgFillWidth, imgStartIndex, imgIndex,
               dataBufferSize, imgBasePath, imgFileType);
    detectKeyPoints(&dataBuffer, feature_detector_name);
    descKeypoints(&dataBuffer, feature_descriptor_name);
    // wait until at least two images have been processed
    if (dataBuffer.size() > 1) {
      string matcherType = "MAT_BF";    // MAT_BF, MAT_FLANN
      string descriptorType;            // DES_BINARY, DES_HOG
      string selectorType = "SEL_KNN";  // SEL_NN, SEL_KNN
      if (binary_descriptors.count(feature_descriptor_name) > 0) {
        descriptorType = "DES_BINARY";
      } else {
        descriptorType = "DES_HOG";
      }
      matchDescriptors(&dataBuffer, descriptorType, matcherType, selectorType);
      // visualize matches between current and previous image
      if (bVis) {
        visMatches(out_dir, feature_detector_name, feature_descriptor_name,
                   dataBuffer);
      }
      // only visualize the first pair of images
      bVis = false;
    }

  }  // eof loop over all images
}

/* MAIN PROGRAM */
int main(int argc, const char* argv[]) {
  //"Feature_detector_name:  SHITOMASI,  HARRIS, FAST, BRISK, ORB,  AKAZE,
  // SIFT
  // "Feature_descriptor_name: BRIEF, ORB, FREAK, AKAZE, SIFT"
  // bool bVis = true;  // visualize results
  string imgBasePath = "../images/";
  string out_dir = "../images/results/";
  vector<string> feature_detector_choices = {
      "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
  vector<string> feature_descriptor_choices = {"BRIEF", "ORB", "FREAK", "AKAZE",
                                               "SIFT"};
  for (string feature_detector_name : feature_detector_choices) {
    for (string feature_descriptor_name : feature_descriptor_choices) {
      std::cout << feature_detector_name << "  " << feature_descriptor_name
                << std::endl;
      //"AKAZE" descriptor only works with "AKAZE" detector
      if ((feature_detector_name == "AKAZE" ||
           feature_descriptor_name == "AKAZE") &&
          feature_descriptor_name != feature_detector_name) {
        continue;
      }
      try {
        featureTrackingPipelinle(imgBasePath, out_dir, feature_detector_name,
                                 feature_descriptor_name);
      } catch (cv::Exception& e) {
        std::cerr << e.msg << std::endl;
      }
    }
  }
  return 0;
}
