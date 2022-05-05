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

## MP.5 Descriptor Matching

The project implements the brute-force and FLANN matchers. The choice can be specified by the `matcherType` string. 

In case of a brute-force matcher, the Hamming distance is used for binary descriptors, whereas for HOG descriptors L2 distance is used.

A keypoint can be matched to one or more keypoints in the next frame. The both methods are selectable using the `selectorType` string ("SEL_NN" or "SEL_KNN").

## MP.6 Descriptor Distance Ratio

Descriptor distance test is used as a filtering method for removing bad keypoint matches. It is implemented via 2-nearest-neighbor matching; 0.8 is a distance ratio threshold. 

## MP.7 Performance Evaluation 1

Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

### The number of keypoints on the preceding vehicle 

Frame   | Harris    | Shi-Tomasi    | FAST  | BRISK | ORB   | AKAZE | SIFT  |
---     |-------    |---            | ---   | ---   | ---   | ---   | ----  |
1       |  45       | 125           | 419   | 264   |  92   | 166   | 138   |
2       |  35       | 118           | 427   | 282   | 102   | 157   | 132   |
3       |  90       | 123           | 404   | 282   | 106   | 161   | 124   |
4       |  48       | 120           | 423   | 277   | 113   | 155   | 137   |
5       |  65       | 120           | 386   | 297   | 109   | 163   | 134   |
6       |  44       | 113           | 414   | 279   | 125   | 164   | 140   |
7       |  63       | 114           | 418   | 289   | 130   | 173   | 137   |
8       |  104      | 123           | 406   | 272   | 129   | 175   | 148   |
9       |  120      | 111           | 396   | 266   | 127   | 177   | 159   |
10      |  98       | 112           | 401   | 254   | 128   | 179   | 137   |
Avg.    |  71       | 117           | 409   | 276   | 116   | 167   | 138   |


### Average size of a keypoint on the preceding vehicle

Frame   | Harris    | Shi-Tomasi    | FAST  | BRISK | ORB   | AKAZE | SIFT  |
---     |-------    |---            | ---   | ---   | ---   | ---   | ----  |
1       | 5         | 4             | 7     |21.5492|57.0723|7.72918|4.98471|
2       | 5         | 4             | 7     |21.7853|57.2273|7.49021|5.0898 |
3       | 5         | 4             | 7     |21.6509|56.4948|7.45212|4.93927|
4       | 5         | 4             | 7     |20.3583|55.1436|7.57523|4.73122|
5       | 5         | 4             | 7     |22.5911|56.7442|7.73319|4.71959|
6       | 5         | 4             | 7     |22.9442|56.6367|7.68804|4.68397|
7       | 5         | 4             | 7     |21.8014|56.7683|7.73879|5.40797|
8       | 5         | 4             | 7     |22.1472|55.4296|7.82613|4.62187|
9       | 5         | 4             | 7     |22.5558|54.6723|7.81556|5.51997|
10      | 5         | 4             | 7     |22.0389|54.3885|7.88576|5.6251 |
Avg.    | 5         | 4             | 7     |21.9422|56.0578|7.69342|5.03235|

### Standard deviation of a keypoint size on the preceding vehicle

Frame   | Harris    | Shi-Tomasi    | FAST  | BRISK | ORB   | AKAZE | SIFT  |
---     |-------    |---            | ---   | ---   | ---   | ---   | ----  |
1       |  0        | 0             |  0    |14.5772|25.7113|3.92182|5.92971|
2       |  0        | 0             |  0    |14.5646|26.0881|3.52456|6.17299|
3       |  0        | 0             |  0    |13.8213|25.9251|3.55275|6.02017|
4       |  0        | 0             |  0    |12.6186|25.0955|3.45896|5.24027|
5       |  0        | 0             |  0    |14.856 |25.013 |3.43779|5.51047|
6       |  0        | 0             |  0    |15.8103|24.4294|3.37843|5.57652|
7       |  0        | 0             |  0    |14.6758|25.4243|3.43237|6.51365|
8       |  0        | 0             |  0    |15.0339|24.7358|3.50975|5.14347|
9       |  0        | 0             |  0    |15.1888|25.2603|3.49352|6.67002|
10      |  0        | 0             |  0    |14.6671|23.6705|3.60137|6.68117|
Avg.    |  0        | 0             |  0    |14.5814|25.1353|3.53113|5.94584|


## MP.8 Performance Evaluation 2

The table below shows the number of matched keypoints averaged over all images for each combination of detectors (on the top) and descriptors (on the left). In the matching step, the brute force approach is used with the descriptor distance ratio set to 0.8.

&       | Harris    | Shi-Tomasi    | FAST  | BRISK | ORB   | AKAZE | SIFT  |
---     |-------    |---            | ---   | ---   | ---   | ---   | ----  |
BRIEF   |  38       |  104          | 314   |  189  | 60    | 140   | 78    |
ORB     |  40       |  100          | 307   |  168  | 84    | 131   | N/A** |
FREAK   |  39       |  85           | 248   |  169  | 46    | 131   | 65    |
AKAZE   |  N/A*     |  N/A*         | N/A*  | N/A*  | N/A*  | 139   | N/A*  |
SIFT    |  34       |  103          | 309   | 182   | 84    | 141   | 88    |

\* AKAZE descriptors can only be used with KAZE or AKAZE keypoints

\** ORB descriptors failed to work with SIFT keypoints in OpenCV 4.1 and 4.5.2 (both on Windows and Ubuntu)


## MP.9 Performance Evaluation 3

The total time (in milliseconds) for keypoint detection and descriptor extraction averaged over all frames is shown for each combination of detectors (on the top) and descriptors (on the left) in the table below:

  &     | Harris    | Shi-Tomasi    | FAST  | BRISK | ORB   | AKAZE | SIFT  |
---     |-------    |---            | ---   | ---   | ---   | ---   | ----  |
BRIEF   |  55       | 50            | 13    | 83    | 29    | 243   | 302   |
ORB     |  47       | 62            | 9     | 85    | 41    | 311   | N/A** |
FREAK   |  188      | 211           | 172   | 267   | 169   | 478   | 393   |
AKAZE   |  N/A*     | N/A*          | N/A*  | N/A*  | N/A*  | 637   | N/A*  |
SIFT    |  235      | 188           | 272   | 549   | 372   | 563   | 1075  |

\* AKAZE descriptors can only be used with KAZE or AKAZE keypoints

\** ORB descriptors failed to work with SIFT keypoints in OpenCV 4.1 and 4.5.2 (both on Windows and Ubuntu)


This data suggests the following top choices of detector/descriptor pairs in terms of speed:
1. FAST keypoints + ORB descriptors
2. FAST keypoints + BRIEF descriptors
3. ORB keypoints + BRIEF descriptors
