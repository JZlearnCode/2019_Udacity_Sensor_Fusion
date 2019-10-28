#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several
// matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource,
                      std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                      cv::Mat &descRef, std::vector<cv::DMatch> &matches,
                      std::string descriptorType, std::string matcherType,
                      std::string selectorType) {
  // configure matcher
  bool crossCheck = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;
  // cv2.NORM_L2: good for SIFT, SURF etc
  // cv2.NORM_HAMMING: binary string based descriptors like ORB, BRIEF, BRISK
  if (matcherType == "MAT_BF") {
    int normType;
    if (descriptorType == "DES_HOG") {
      normType = cv::NORM_L2;
    } else if (descriptorType == "DES_BINARY") {
      normType = cv::NORM_HAMMING;
    } else {
      std::cerr << "Wrong descriptorType" << std::endl;
    }
    matcher = cv::BFMatcher::create(normType, crossCheck);
  }

  else if (matcherType == "MAT_FLANN") {
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  }

  // perform matching task
  if (selectorType.compare("SEL_NN") == 0) {  // nearest neighbor (best match)
    matcher->match(
        descSource, descRef,
        matches);  // Finds the best match for each descriptor in desc1
  }
  // k nearest neighbors (k=2)
  else if (selectorType.compare("SEL_KNN") == 0) {
    std::vector<std::vector<cv::DMatch> > knn_matches;
    matcher->knnMatch(descSource, descRef, knn_matches, 2);
    //-- Filter matches using the Lowe's ratio test
    // The probability that a match is correct can be determined by taking the
    // ratio of distance from the closest neighbor to the distance of the second
    // closest
    const float ratio_thresh = 0.7f;
    for (size_t i = 0; i < knn_matches.size(); i++) {
      if (knn_matches[i][0].distance <
          ratio_thresh * knn_matches[i][1].distance) {
        matches.push_back(knn_matches[i][0]);
      }
    }
  }
}

// Use one of several types of state-of-art descriptors to uniquely identify
// keypoints: BRIEF, ORB, FREAK, AKAZE, SIFT
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                   cv::Mat &descriptors, string descriptorType) {
  // select appropriate descriptor
  cv::Ptr<cv::DescriptorExtractor> extractor;
  if (descriptorType == "BRISK") {
    int threshold = 30;         // FAST/AGAST detection threshold score.
    int octaves = 3;            // detection octaves (use 0 to do single scale)
    float patternScale = 1.0f;  // apply this scale to the pattern used for
                                // sampling the neighbourhood of a keypoint.

    extractor = cv::BRISK::create(threshold, octaves, patternScale);
  } else if (descriptorType == "BRIEF") {
    extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
  } else if (descriptorType == "ORB") {
    extractor = cv::ORB::create();
  } else if (descriptorType == "FREAK") {
    extractor = cv::xfeatures2d::FREAK::create();
  } else if (descriptorType == "AKAZE") {
    extractor = cv::AKAZE::create();
  } else if (descriptorType == "SIFT") {
    extractor = cv::xfeatures2d::SIFT::create();
  } else {
    std::cerr << "Invalid Keypoint Descriptor Type: " << descriptorType
              << std::endl;
  }

  // perform feature description
  double t = (double)cv::getTickCount();

  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0
       << " ms" << endl;
}

// FAST, BRISK, ORB, AKAZE, SIFT
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                        std::string detectorType) {
  cv::Ptr<cv::FeatureDetector> detector;
  if (detectorType == "FAST") {
    detector = cv::FastFeatureDetector::create();
  } else if (detectorType == "BRISK") {
    detector = cv::BRISK::create();
  } else if (detectorType == "ORB") {
    detector = cv::ORB::create();
  } else if (detectorType == "AKAZE") {
    detector = cv::AKAZE::create();
  } else if (detectorType == "SIFT") {
    detector = cv::xfeatures2d::SIFT::create();
  } else {
    std::cerr << "Invalid Detector Type: " << detectorType << std::endl;
    return;
  }
  detector->detect(img, keypoints);
}

bool compareKeypointsHarris(cv::KeyPoint p1, cv::KeyPoint p2) {
  return (p1.response > p2.response);
}
// For each pixel (x, y), it calculates a 2x2 gradient covariance matrix M(x, y)
// over a "blocksize x blocksize" neighborhood
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  int blockSize = 2;
  // kernel size for Sobel filter, must be odd
  int kernelSize = 3;
  // Harris corner free parmeter
  double k = 0.04;
  float thresh = 100;
  cv::Mat corner_img;
  cv::cornerHarris(img, corner_img, blockSize, kernelSize, k,
                   cv::BORDER_DEFAULT);
  normalize(corner_img, corner_img, 0, 255, cv::NORM_MINMAX, CV_32FC1);
  // add corners to result vector
  for (int row = 0; row < corner_img.rows; row++) {
    for (int col = 0; col < corner_img.cols; col++) {
      float response = corner_img.at<float>(row, col);
      if (response > thresh) {
        cv::KeyPoint corner_point;
        corner_point.pt = cv::Point2f(col, row);
        // diameter of the meaningful keypoint neighborhood
        corner_point.size = blockSize;
        corner_point.response = response;
        keypoints.push_back(corner_point);
      }
    }
  }
  // sort by corner response
  sort(keypoints.begin(), keypoints.end(), compareKeypointsHarris);
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  // compute detector parameters based on image size
  int blockSize = 4;  //  size of an average block for computing a derivative
                      //  covariation matrix over each pixel neighborhood
  double maxOverlap =
      0.0;  // max. permissible overlap between two features in %
  double minDistance = (1.0 - maxOverlap) * blockSize;
  int maxCorners =
      img.rows * img.cols / max(1.0, minDistance);  // max. num. of keypoints

  double qualityLevel = 0.01;  // minimal accepted quality of image corners
  double k = 0.04;

  // Apply corner detection
  double t = (double)cv::getTickCount();
  vector<cv::Point2f> corners;
  // goodFeaturesToTrack is the Shi-Tomasi corner detector
  cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance,
                          cv::Mat(), blockSize, false, k);

  // add corners to result vector
  for (auto it = corners.begin(); it != corners.end(); ++it) {
    cv::KeyPoint newKeyPoint;
    newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
    newKeyPoint.size = blockSize;
    keypoints.push_back(newKeyPoint);
  }
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in "
       << 1000 * t / 1.0 << " ms" << endl;
}