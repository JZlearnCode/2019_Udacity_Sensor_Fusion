#include "inputOutputUtil.h"

using namespace std;

void detectLidar(string imgBasePath, string lidarPrefix, string imgNumber, string lidarFileType,
                 cv::Mat P_rect_00, cv::Mat R_rect_00, cv::Mat RT,
                 std::deque<DataFrame>* dataBuffer) {
    // load 3D Lidar points from file
    string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber + lidarFileType;
    std::vector<LidarPoint> lidarPoints;
    loadLidarFromFile(lidarPoints, lidarFullFilename);

    // remove Lidar points based on distance properties
    float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
    cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
    (dataBuffer->end() - 1)->lidarPoints = lidarPoints;

    /* CLUSTER LIDAR POINT CLOUD */
    // associate Lidar points with camera-based ROI
    float shrinkFactor = 0.2; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
    clusterLidarWithROI((dataBuffer->end()-1)->boundingBoxes, 
                        (dataBuffer->end() - 1)->lidarPoints, 
                        shrinkFactor, P_rect_00, R_rect_00, RT);
}

void calculateTTC(std::deque<DataFrame>* dataBuffer, double sensorFrameRate,  
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
/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */
    // data location
    std::string dataPath = "../";
    // camera
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;  // last file index to load
    int imgStepWidth = 1; 
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    std::string yoloBasePath = dataPath + "dat/yolo/";
    std::string yoloClassesFile = yoloBasePath + "coco.names";
    std::string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    std::string yoloModelWeights = yoloBasePath + "yolov3.weights";
    // parameters for yolo object detection
    float confThreshold = 0.2; 
    float nmsThreshold = 0.4;

    // Lidar
    std::string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    std::string lidarFileType = ".bin";

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    std::deque<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = true;            // visualize results

    // 2D feature detection parameters
    string feature_detector_name = "SIFT";
    string feature_descriptor_name = "SIFT"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
    string descriptorType = "DES_HOG"; // DES_BINARY, DES_HOG
    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    loadCalibrationParams(P_rect_00, R_rect_00, RT);

    // sum of difference for lidarTTC and cameraTTC
    vector<float> distDifference; 
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
                    P_rect_00, R_rect_00, RT, &dataBuffer);    
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
            calculateTTC(&dataBuffer, sensorFrameRate,  
                         P_rect_00, R_rect_00, RT, &distDifference, bVis); 
        }
    } // eof loop over all images
    float sumDistDifference = accumulate(distDifference.begin(), distDifference.end(), 0); 
    float avgDistDifference = sumDistDifference / distDifference.size();
    std::cout<<"avgDistDifference"<< avgDistDifference <<std::endl;

    return 0;
}
