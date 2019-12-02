#include "inputOutputUtil.hpp"

using namespace std;
void loadCalibrationParams(cv::Mat P_rect_00, cv::Mat R_rect_00, cv::Mat RT) {
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    
}

string loadImages(deque<DataFrame>* dataBuffer, string imgPrefix,
                int imgFillWidth, int imgStartIndex, int imgIndex,
                int dataBufferSize, string imgBasePath, string imgFileType) {
  // assemble filenames for current index
  ostringstream imgNumber;
  imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
  string imgFullFilename =
      imgBasePath + imgPrefix + imgNumber.str() + imgFileType;
  // load image from file and convert to grayscale
  cv::Mat img = cv::imread(imgFullFilename);
  // ring buffer of size dataBufferSize
  DataFrame frame;
  frame.cameraImg = img;
  dataBuffer->push_back(frame);
  // remove old images
  if (dataBuffer->size() > dataBufferSize) {
    dataBuffer->pop_front();
  }
  return imgNumber.str();
}

void visResult(const deque<DataFrame>& dataBuffer, BoundingBox* currBB, bool bVis,
               cv::Mat& P_rect_00, cv::Mat& R_rect_00, cv::Mat& RT,
               double ttcLidar, double ttcCamera) {
    if (bVis)
    {
        cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
        showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
        cv::rectangle(visImg, 
                      cv::Point(currBB->roi.x, currBB->roi.y), 
                      cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), 
                      cv::Scalar(0, 255, 0), 2);
        
        char str[200];
        sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
        putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));

        string windowName = "Final Results : TTC";
        cv::namedWindow(windowName, 4);
        cv::imshow(windowName, visImg);
        cout << "Press key to continue to next frame" << endl;
        cv::waitKey(0);
    }
}