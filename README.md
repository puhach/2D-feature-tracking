# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

## MP.1 Data Buffer Optimization

The ring buffer is implemented usign `std::deque`, which is an indexed sequence container that allows fast insertion and deletion at both its beginning and its end. The size of the data buffer never exceeds a limit (e.g. 2 elements). This is achieved by pushing in new elements to the end of the queue and removing existing elements from the beginning of the queue.

## MP.2 Keypoint Detection

The project makes use of the following detectors available in OpenCV:
* Harris
* Shi-Tomasi
* FAST 
* BRISK 
* ORB
* AKAZE
* SIFT 

The Harris detctor is implemented in the `detKeypointsHarris` function, Shi-Tomasi detector was provided in `detKeypointsShiTomasi`, and the rest of keypoint detectors are implemented in `detKeypointsModern`. In order to choose a desired detector, the `detectorType` string should be set.

## MP.3 Keypoint Removal

The keypoints are filtered out unless they belong to the preceding vehicle's bounding box specified as [535, 180, 180, 150]. Further processing is performed for the remaining keypoints only.

## MP.4 Keypoint Descriptors

The project exploits the following descriptors from OpenCV:
* BRIEF 
* ORB 
* FREAK
* AKAZE 
* SIFT 

Descriptor extraction is implemented in the function `descKeypoints`. The `descriptorType` string is used for selecting a descriptor type.

