#include <unordered_set>
#include "inputOutputUtil.hpp"

using namespace std;

void ttcPipeline(string feature_detector_name,
                string feature_descriptor_name)
{
    /* INIT VARIABLES AND DATA STRUCTURES */
    // data location
    // camera
    std::string imgBasePath = "../data/";
    std::string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;  // last file index to load
    int imgStepWidth = 1; 
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    std::string yoloBasePath = "../data/yolo/";
    std::string yoloClassesFile = yoloBasePath + "coco.names";
    std::string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    std::string yoloModelWeights = yoloBasePath + "yolov3.weights";
    // parameters for yolo object detection
    float confThreshold = 0.2; 
    float nmsThreshold = 0.4;
    // Feature detection
    unordered_set<string> binary_descriptors = {"BRIEF", "ORB", "FREAK", "AKAZE"};
    string matcherType = "MAT_BF";    // MAT_BF, MAT_FLANN
    string descriptorType;            // DES_BINARY, DES_HOG
    string selectorType = "SEL_KNN";  // SEL_NN, SEL_KNN
    if (binary_descriptors.count(feature_descriptor_name) > 0) {
        descriptorType = "DES_BINARY";
    } else {
        descriptorType = "DES_HOG";
    }


    // Lidar
    std::string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    std::string lidarFileType = ".bin";

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    std::deque<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    loadCalibrationParams(P_rect_00, R_rect_00, RT);

    // sum of difference for lidarTTC and cameraTTC
    vector<float> ttcCameraVect;
    vector<float> ttcLidarVect;
    vector<float> ttcDifference; 
    /* MAIN LOOP OVER ALL IMAGES */
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
    {
        /* LOAD IMAGE INTO BUFFER */
        string imgNumber =  
        loadImages(&dataBuffer, imgPrefix, imgFillWidth, imgStartIndex, imgIndex,
                   dataBufferSize, imgBasePath, imgFileType);
        /* DETECT & CLASSIFY OBJECTS */        
        detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, 
                       confThreshold, nmsThreshold, yoloBasePath, yoloClassesFile, 
                       yoloModelConfiguration, yoloModelWeights, bVis);
        /* CROP LIDAR POINTS */
        detectLidar(imgBasePath, lidarPrefix, imgNumber, lidarFileType,
                    P_rect_00, R_rect_00, RT, &dataBuffer, bVis);    
        /* DETECT IMAGE KEYPOINTS */
        // extract 2D keypoints from current image
        detectKeyPoints(&dataBuffer, feature_detector_name);
        /* EXTRACT KEYPOINT DESCRIPTORS */
        descKeypoints(&dataBuffer, feature_descriptor_name);
        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {
            /* MATCH KEYPOINT DESCRIPTORS */
            matchDescriptors(&dataBuffer, descriptorType, matcherType, selectorType);
            /* TRACK 3D OBJECT BOUNDING BOXES */
            // associate bounding boxes between current and previous frame using keypoint matches
            matchBoundingBoxes(&dataBuffer);
            /* COMPUTE TTC ON OBJECT IN FRONT */  
            calculateTTCCombined(&dataBuffer, sensorFrameRate,  
                         P_rect_00, R_rect_00, RT, 
                         &ttcCameraVect, &ttcLidarVect, &ttcDifference, bVis); 
        }
    } // eof loop over all images
    float avgTTCdifference = accumulate(ttcDifference.begin(), ttcDifference.end(), 0) / float(ttcDifference.size()); 
    float avgTTCcamera = accumulate(ttcCameraVect.begin(), ttcCameraVect.end(), 0) / float(ttcCameraVect.size()); 
    float avgTTClidar = accumulate(ttcLidarVect.begin(), ttcLidarVect.end(), 0) / float(ttcLidarVect.size()); 

    std::cout<<"feature_detector_name "<< feature_detector_name 
    <<" feature_descriptor_name "<<feature_descriptor_name
    <<" avgTTCcamera "<<avgTTCcamera 
    <<" avgTTClidar "<<avgTTClidar
    <<" avgTTCdifference "<<avgTTCdifference<<std::endl;
    return;
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
        ttcPipeline(feature_detector_name, feature_descriptor_name);
      } catch (cv::Exception& e) {
        std::cerr << e.msg << std::endl;
      }
    }
  }
  return 0;
}