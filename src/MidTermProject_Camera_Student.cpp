/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <deque>
#include <cmath>
#include <limits>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>

/// check whether we need to include xfeatures2d to use SIFT
//#if CV_VERSION_MAJOR*10+CV_VERSION_MINOR < 44 
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
//#endif

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    //vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    std::deque<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    unsigned long totalKeypoints = 0;
    float avgKeyPointSize = 0.0f;
    float avgKeyPointSizeStdDev = 0.0f;
    unsigned long avgMatchNum = 0;
    double avgDetectionTime = 0, avgDescriptionTime = 0;

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        //dataBuffer.push_back(frame);
        
        // std::deque seems to be more appropriate for this task as compared to std::vector;
        // ideally, we don't even need a container here, but the existing code implies using it
        if (dataBuffer.size() >= dataBufferSize)
            dataBuffer.pop_front();
            //dataBuffer.erase(dataBuffer.begin());
        
        dataBuffer.push_back(std::move(frame));
        assert(dataBuffer.size() <= dataBufferSize);

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        string detectorType = /*"ORB";*/ /*"BRISK";*/ "FAST"; /*"HARRIS";*/ /*"SHITOMASI";*/ /*"AKAZE"*/ /*"SIFT"*/;

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        double detectionTime = 0;
        if (detectorType.compare("SHITOMASI") == 0)
        {
            detectionTime = detKeypointsShiTomasi(keypoints, imgGray, false);
        }   // SHITOMASI
        else if (detectorType.compare("HARRIS") == 0)
        {
            detectionTime = detKeypointsHarris(keypoints, imgGray, false);
        }   // HARRIS
        else
        {
            detectionTime = detKeypointsModern(keypoints, imgGray, detectorType, false);
        }   // other detector types


        std::cout << detectorType << " detection with n = " << keypoints.size() << " keypoints in " << 1000 * detectionTime << " ms" << std::endl;

        avgDetectionTime += detectionTime;

        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            keypoints.erase(
                std::remove_if(std::begin(keypoints), std::end(keypoints), 
                    [&vehicleRect](const auto& kp) 
                    {
                        return !vehicleRect.contains(kp.pt);
                        /*return kp.pt.x < vehicleRect.tl().x
                            || kp.pt.y < vehicleRect.tl().y
                            || kp.pt.x > vehicleRect.br().x
                            || kp.pt.y > vehicleRect.br().y;*/
                    }), 
                std::cend(keypoints));
        }   // bFocusOnVehicle

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false; 
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // count the number of keypoinst detected in all images
        totalKeypoints += static_cast<unsigned long>(keypoints.size());

        // compute the average size of a keypoint
        auto frameAvgKeyPointSize = std::accumulate(std::cbegin(keypoints), std::cend(keypoints), 0.0f, 
            [](auto acc, const auto& kp) 
            {
                return acc + kp.size;
            });

        frameAvgKeyPointSize /= keypoints.size() + std::numeric_limits<float>::epsilon();
        
        avgKeyPointSize += frameAvgKeyPointSize;

        // compute the standard deviation
        auto frameKeyPointSizeStdDev = 0.0f;
        for (const auto& kp : keypoints)
        {
            auto szDiff = kp.size - frameAvgKeyPointSize;
            frameKeyPointSizeStdDev += szDiff * szDiff;
        }

        frameKeyPointSizeStdDev = std::sqrt(frameKeyPointSizeStdDev / (keypoints.size() + std::numeric_limits<float>::epsilon()));
        
        avgKeyPointSizeStdDev += frameKeyPointSizeStdDev;

        std::cout << "Vehicle keypoint number: " << keypoints.size() << std::endl;
        std::cout << "Mean vehicle keypoint size: " << frameAvgKeyPointSize << std::endl;
        std::cout << "Standard deviation: " << frameKeyPointSizeStdDev << std::endl;

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        //string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
        std::string descriptorType = /*"AKAZE";*/ /*"BRIEF";*/ /*"SIFT";*/ /*"FREAK";*/ "ORB";
        double descriptionTime = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

        cout << descriptorType << " descriptor extraction in " << 1000 * descriptionTime << " ms" << endl;

        avgDescriptionTime += descriptionTime;

        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = /*"MAT_FLANN";*/ "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorType = "DES_HOG"; /*"DES_BINARY";*/ // DES_BINARY, DES_HOG
            string selectorType = "SEL_KNN"; /*"SEL_NN"*/;       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            std::cout << "Number of matches: " << matches.size() << std::endl;

            avgMatchNum += static_cast<unsigned long>(matches.size());

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

        std::cout << std::endl;
    } // eof loop over all images

    auto numFrames = (imgEndIndex - imgStartIndex + 1);
    auto avgKeyPointNumPerFrame = totalKeypoints / numFrames;
    avgKeyPointSize /= numFrames;
    avgKeyPointSizeStdDev /= numFrames;
    
    // the number of matches can only be counted between two frames, therefore the first frame is skipped
    if (numFrames >= 2)
        avgMatchNum /= (numFrames - 1);
    else
        avgMatchNum = 0;

    avgDetectionTime /= numFrames;
    avgDescriptionTime /= numFrames;

    std::cout << "Average number of vehicle keypoints per frame: " << avgKeyPointNumPerFrame << std::endl;
    std::cout << "Average vehicle keypoint size per frame: " << avgKeyPointSize << std::endl;
    std::cout << "Vehicle keypoint size standard deviation per frame: " << avgKeyPointSizeStdDev << std::endl;
    std::cout << "Average number of matches: " << avgMatchNum << std::endl;
    std::cout << "Average keypoint detection time: " << avgDetectionTime*1000 << " ms" << std::endl;
    std::cout << "Average descriptor extraction time: " << avgDescriptionTime*1000 << " ms" << std::endl;
    std::cout << "Average processing time: " << (avgDetectionTime+avgDescriptionTime)*1000 << " ms" << std::endl;
    return 0;
}