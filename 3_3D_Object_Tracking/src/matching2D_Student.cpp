
#include <numeric>
#include "matching2D.hpp"

using namespace std;

cv::Ptr<cv::DescriptorMatcher> createMatcher(std::string descriptorType,
                                             std::string matcherType) {
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
    matcher = cv::BFMatcher::create(normType);
  }

  else if (matcherType == "MAT_FLANN") {
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  }
  return matcher;
}

// TODO: Code from Judy's implementation of project2 
// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    // configure matcher
    cv::Ptr<cv::DescriptorMatcher> matcher = createMatcher(descriptorType, matcherType);

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
        const float ratio_thresh = 0.8f;
        for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance <
            ratio_thresh * knn_matches[i][1].distance) {
            matches.push_back(knn_matches[i][0]);
        }
        }
    }
    cout << "Number of matched keypoints = " << matches.size() << endl;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else
    {

        //...
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}