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
  // evaluation of algorithm performance
  double sum_detector_time = 0;
  double sum_descriptor_time = 0;
  double sum_num_keypoints = 0;
  double sum_num_matches = 0;

  /* MAIN LOOP OVER ALL IMAGES */
  for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex;
       imgIndex++) {
    loadImages(&dataBuffer, imgPrefix, imgFillWidth, imgStartIndex, imgIndex,
               dataBufferSize, imgBasePath, imgFileType);
    double start_detector = (double)cv::getTickCount();
    detectKeyPoints(&dataBuffer, feature_detector_name);
    // calculate detector time,  seconds --> milliseconds
    double detector_time = ((double)cv::getTickCount() - start_detector) /
                           cv::getTickFrequency() * 1000;
    sum_detector_time += detector_time;
    double start_descriptor = (double)cv::getTickCount();
    descKeypoints(&dataBuffer, feature_descriptor_name);
    // calculate descriptor time,  seconds --> milliseconds
    double descriptor_time = ((double)cv::getTickCount() - start_descriptor) /
                             cv::getTickFrequency() * 1000;
    sum_descriptor_time += descriptor_time;
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

      // number of keypoints detected in each frame
      sum_num_keypoints += (dataBuffer.back()).keypoints.size();
      // number of matches found
      sum_num_matches += (dataBuffer.back()).kptMatches.size();
      // Visualize matches between current and previous image
      if (bVis) {
        visMatches(out_dir, feature_detector_name, feature_descriptor_name,
                   dataBuffer);
      }
      // only visualize the first pair of images
      bVis = false;
    }
  }  // eof loop over all images
  double num_images = imgEndIndex - imgStartIndex + 1;
  double average_detector_time = sum_detector_time / num_images;
  double average_descriptor_time = sum_descriptor_time / num_images;
  double average_num_keypoints =
      sum_num_keypoints / (num_images - dataBufferSize + 1);
  double average_num_matches =
      sum_num_matches / (num_images - dataBufferSize + 1);
  std::cout << "feature detector:" << feature_detector_name
            << "  feature descriptor: " << feature_descriptor_name << std::endl;
  std::cout << "average detector time " << average_detector_time
            << " milliseconds" << std::endl;
  std::cout << "average descriptor time " << average_descriptor_time
            << " milliseconds" << std::endl;
  std::cout << "average num keypoints detected per frame "
            << average_num_keypoints << std::endl;
  std::cout << "average number of keypoint matches " << average_num_matches
            << std::endl;
  std::cout << "--------------------------------------------" << std::endl;
}

/* MAIN PROGRAM */
int main(int argc, const char* argv[]) {
  string imgBasePath = "../images/";
  string out_dir = "../images/results/";
  vector<string> feature_detector_choices = {
      "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
  vector<string> feature_descriptor_choices = {"BRIEF", "ORB", "FREAK", "AKAZE",
                                               "SIFT"};
  for (string feature_detector_name : feature_detector_choices) {
    for (string feature_descriptor_name : feature_descriptor_choices) {
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
