#include "util.h"
using namespace std;

// Load images into buffer
void loadImages(deque<DataFrame>* dataBuffer, string imgPrefix,
                int imgFillWidth, int imgStartIndex, int imgIndex,
                int dataBufferSize, string imgBasePath, string imgFileType) {
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
  dataBuffer->push_back(frame);
  // remove old images
  if (dataBuffer->size() > dataBufferSize) {
    dataBuffer->pop_front();
  }
}

void visMatches(string out_dir, string feature_detector_name,
                string feature_descriptor_name, deque<DataFrame>& dataBuffer) {
  cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
  cv::drawMatches(
      (dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
      (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
      (dataBuffer.end() - 1)->kptMatches, matchImg, cv::Scalar::all(-1),
      cv::Scalar::all(-1), vector<char>(),
      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  string result_image_path =
      out_dir + feature_detector_name + "_" + feature_descriptor_name + ".png";
  std::cout << "result_image_path" << result_image_path << std::endl;
  cv::imwrite(result_image_path, matchImg);
}