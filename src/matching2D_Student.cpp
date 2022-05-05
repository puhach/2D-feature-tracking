#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType.compare("DES_BINARY")==0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        // Workaround for the OpenCV bug
        
        if (descSource.type() != CV_32F)
            descSource.convertTo(descSource, CV_32F);
        
        if (descRef.type() != CV_32F)
            descRef.convertTo(descRef, CV_32F);
        
        matcher = cv::FlannBasedMatcher::create();
    }
    else throw std::runtime_error("Unknown matcher type: " + matcherType);

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        std::vector<std::vector<cv::DMatch>> kmatches;
        matcher->knnMatch(descSource, descRef, kmatches, 2);

        matches.clear();
        for (auto&& neighbors : kmatches)
        {
            assert(!neighbors.empty());
            if (neighbors.size() < 2 || neighbors[0].distance < 0.8*neighbors[1].distance)
            {
                matches.push_back(std::move(neighbors[0]));
            }
        }   // for neighbors
    }   // SEL_KNN
    else throw std::runtime_error("Unknown selector type: " + selectorType);
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
double descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
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
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }   // BRIEF
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }   // ORB
    else if (descriptorType.compare("FREAK") == 0)
    {        
        extractor = cv::xfeatures2d::FREAK::create();
    }   // FREAK
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }   // AKAZE
    else if (descriptorType.compare("SIFT") == 0)
    {
#if CV_VERSION_MAJOR*10+CV_VERSION_MINOR < 44
        extractor = cv::xfeatures2d::SiftDescriptorExtractor::create();
#else
        extractor = cv::SiftDescriptorExtractor::create();
#endif
    }   // SIFT
    else throw std::runtime_error("Unknown descriptor type: " + descriptorType);

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    return t;
}   // descKeypoints

double detKeypointsHarris(std::vector<cv::KeyPoint>& keypoints, cv::Mat& img, bool bVis)
{
    double t = static_cast<double>(cv::getTickCount());

    int blockSize = 5;
    int apertureSize = 3;
    double k = 0.05;
    cv::Mat imgCorners;
    cv::cornerHarris(img, imgCorners, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

    cv::Mat imgCornersNorm;
    cv::normalize(imgCorners, imgCornersNorm, 0, 255, cv::NORM_MINMAX);

    float minCornerness = 100;
    keypoints.clear();
    for (int i = 0; i < imgCornersNorm.rows; ++i)
    {
        for (int j = 0; j < imgCornersNorm.cols; ++j)
        {
            float harrisResponse = imgCornersNorm.at<float>(i, j);
            if (harrisResponse >= minCornerness)
            {
                keypoints.emplace_back(
                    static_cast<float>(j), static_cast<float>(i), 
                    static_cast<float>(blockSize), -1.0f, harrisResponse);
            }
        }
    }

    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();

    //std::cout << "Harris detection with n = " << keypoints.size() << " keypoints in " << 1000 * t << " ms" << std::endl;

    if (bVis)
    {
        cv::Mat visImage; // = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        string windowName = "Harris Corner Detector Results";
        //cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey();
    }

    return t;
}   // detKeypointsHarris

// Detect keypoints in image using the traditional Shi-Thomasi detector
double detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = static_cast<int>(img.rows * img.cols / max(1.0, minDistance)); // max. num. of keypoints

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
        newKeyPoint.size = static_cast<float>(blockSize);
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey();
    }

    return t;
}   // detKeypointsShiTomasi

double detKeypointsModern(std::vector<cv::KeyPoint>& keypoints, cv::Mat& img, std::string detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector;

    if (detectorType.compare("FAST") == 0)
    {
        detector = cv::FastFeatureDetector::create();
    }   // FAST
    else if (detectorType.compare("BRISK") == 0)
    {
        detector = cv::BRISK::create();
    }   // BRISK
    else if (detectorType.compare("ORB") == 0)
    {
        detector = cv::ORB::create();
    }   // ORB
    else if (detectorType.compare("AKAZE") == 0)
    {
        detector = cv::AKAZE::create();
    }   // AKAZE
    else if (detectorType.compare("SIFT") == 0)
    {
#if CV_VERSION_MAJOR*10+CV_VERSION_MINOR < 44
        detector = cv::xfeatures2d::SiftFeatureDetector::create();
#else
        detector = cv::SiftFeatureDetector::create();
#endif
    }   // SIFT
    else throw std::runtime_error("Unknown detector type: " + detectorType);

    keypoints.clear();

    double t = static_cast<double>(cv::getTickCount());
    detector->detect(img, keypoints);
    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();

    //std::cout << detectorType << " detection with n = " << keypoints.size() << " in " << t * 1000 << " ms" << std::endl;

    if (bVis)
    {
        cv::Mat visImage;
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        
        string windowName = detectorType + " Detector Results";
        //cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey();
    }

    return t;
}   // detKeypointsModern
