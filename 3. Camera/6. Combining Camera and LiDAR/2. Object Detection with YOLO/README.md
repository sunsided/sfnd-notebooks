# Object Detection with YOLO

![](.readme/result1.jpg)

## Why Object Detection?

As mentioned in the introductory video, we need a way to detect vehicles in our images so that we can isolate matched keypoints as well as projected Lidar points and associate them to a specific object. Let us take a look at the program flow schematic we already discussed in the lesson on engineering a collision detection system.

![](.section/draggedimage.png)

Based on what you learned in the previous lesson, you are now able to detect and match keypoints using a variety of detectors and descriptors. In order to compute the time-to-collision for a specific vehicle however, we need to isolate the keypoints on that vehicle so that TTC estimation is not distorted due to the inclusion of matches on e.g. the road surface, stationary objects or other vehicles in the scene. One way to achieve this is to automatically identify the vehicles in the scene using object detection. The output of such an algorithm would (ideally) be a set of 2D bounding boxes around all objects in the scene. Based on these bounding boxes, we could then easily associate keypoint matches to objects and achieve a stable TTC estimate.

For the Lidar measurements, the same rationale can be applied. As you have learned in the previous course on Lidar, you already know that 3D points can be successfully clustered into individual objects, e.g. by using algorithms from the Point Cloud Library (PCL), which can be seen as an equivalent to the OpenCV library for 3D applications. In this section, let us look at yet another approach to group Lidar points into objects. Based on what you learned in the previous section, you now know how to project Lidar points onto the image plane. Given a set of bounding boxes from object detection, we could thus easily associate a 3D Lidar point to a specific object in a scene by simply checking wether it is enclosed by a bounding box when projected into the camera.

We will look in detail at both of the described approaches later in this section but for now, let us look at a way to detect objects in camera images - which is a prerequisite for grouping keypoint matches as well as Lidar points. in the schematic shown above, the content of the current lesson is highlighted by a blue rectangle and in this section we will be focussing on 'detecting & classifying objects‘.

## Introduction into YOLO

The aim of this section is to enable you to quickly leverage a powerful and state-of-the-art tool for object detection. It is not the purpose to perform a theoretical deep-dive into the inner workings of such algorithms, but rather you should be enabled to integrate object detection into the code framework of this course quickly and seamlessly. The following image shows the an example output of the code we will be developing in this section.

![](.section/draggedimage-1.png)

In the past, methods for object detection were often based on histograms of oriented gradients (HOG) and support vector machines (SVM). Until the advent of deep-learning, the HOG/SVM approach was long considered the state-of-the-art approach to detection. While the results of HOG/SVM are still acceptable for a large variety of problems, its use on platforms with limited processing speed is limited. As with SIFT, one of the major issues is the methods reliance on the intensity gradient - which is a costly operation.

Another approach to object detection is the use of deep-learning frameworks such as TensorFlow or Caffe. However, the learning curve for both is very steep and would warrant an entire course on its own. An easy-to-use alternative that works right out of the box and is also based on similar underlying concepts is YOLO, a very fast detection framework that is shipped with the OpenCV library. Developed by Joseph Redmon, Santosh Divvala, Ross Girshick, and Ali Farhadi at Cornell University, YOLO uses a different approach than most other methods: Here, a single neural network is applied to the full image. This network divides the image into regions and predicts bounding boxes and probabilities for each region. These bounding boxes are weighted by the predicted probabilities. The following figure illustrates the principle:

![](.section/yolo-workflow.png)

Other than classifier-based systems such as HOG/SVM, YOLO looks at the whole image so its predictions are informed by global context in the image. It also makes predictions with a single network pass unlike systems like R-CNN which require thousands of passes for a single image. This makes it extremely fast while at the same time generating similar results as other state-of-the-art methods such as the Single Shot MultiBox Detector (SSD).

The makers of YOLO have also made available a set of pre-trained weights that enable the current version YOLOv3 to recognize 80 different objects in images and videos based on [COCO](http://cocodataset.org/#home), which is a is large-scale object detection, segmentation, and captioning dataset. In the context of this course this means, that we can use YOLOv3 as an out-of-the-box classifier which is able to detect vehicles (and many other obstacles) with reasonable accuracy.

## The YOLOv3 Workflow

In this section, we will take a look at the different steps involved to execute YOLO on the course image set. The parameters used below are mainly the ones suggested by the authors. In the following, a short overview of the main algorithmic steps is given:

- First, the image is divided into a 13x13 grid of cells. Based on the size of the input image, the size of these cells in pixels varies. In the code below, a size of 416 x 416 pixels was used, leading to a cell size of 32 x 32 pixels.
- As seen in the schematic above, each cell is then used for predicting a set of bounding boxes. For each bounding box, the network also predicts the confidence that the bounding box encloses a particular object as well as the probability of the object belonging to a particular class (taken from the COCO dataset).
- Lastly, a non-maximum suppression is used to eliminate bounding boxes with a low confidence level as well as redundant bounding boxes enclosing the same object.

In the following, the main workflow as well as the respective code is presented.

### Step 1: Initialize the Parameters

Every bounding box predicted by YOLOv3 is associated with a confidence score. The parameter 'confThreshold' is used to remove all bounding boxes with a lower score value.

Then, a non-maximum suppression is applied to the remaining bounding boxes. The NMS procedure is controlled by the parameter ‚nmsThreshold‘.

The size of the input image is controlled by the parameters ‚inpWidth‘ and ‚inpHeight‘, which is set to 416 as proposed by the YOLO authors. Other values could e.g. be 320 (faster) or 608 (more accurate).

### Step 2: Prepare the Model

The file `yolov3.weights` contains the pre-trained network’s weights and has been made available by the authors of YOLO [here](https://pjreddie.com/media/files/yolov3.weights).

The file `yolov3.cfg` containing the network configuration is available [here](https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg) and the coco.names file which contains the 80 different class names used in the COCO dataset can be downloaded [here](https://github.com/pjreddie/darknet/blob/master/data/coco.names).

The following code shows how to load the model weights as well as the associated model configuration:

```cpp
// load image from file
cv::Mat img = cv::imread("./images/img1.png");

// load class names from file
string yoloBasePath = "./dat/yolo/";
string yoloClassesFile = yoloBasePath + "coco.names";
string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
string yoloModelWeights = yoloBasePath + "yolov3.weights"; 

vector<string> classes;
ifstream ifs(yoloClassesFile.c_str());
string line;
while (getline(ifs, line)) classes.push_back(line);

// load neural network
cv::dnn::Net net = cv::dnn::readNetFromDarknet(yoloModelConfiguration, yoloModelWeights);
net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
```

After loading the network, the DNN backend is set to `DNN_BACKEND_OPENCV`. If OpenCV is built with Intel’s Inference Engine, `DNN_BACKEND_INFERENCE_ENGINE` should be used instead. The target is set to CPU in the code, as opposed to using `DNN_TARGET_OPENCL`, which would be the method of choice if a (Intel) GPU was available.

### Step 3: Generate 4D Blob from Input Image

As data flows through the network, YOLO stores, communicates, and manipulates the information as "blobs": the blob is the standard array and unified memory interface for many frameworks, including Caffe. A blob is a wrapper over the actual data being processed and passed along and also provides synchronization capability between the CPU and the GPU. Mathematically, a blob is an N-dimensional array stored in a C-contiguous fashion. The conventional blob dimensions for batches of image data are number N x channel C x height H x width W. In this nomenclature, N is the batch size of the data. Batch processing achieves better throughput for communication and device processing. For a training batch of 256 images, N would be 256. The parameter C represents the feature dimension, e.g. for RGB images C = 3. In OpenCV, blobs are stored as 4-dimensional `cv::Mat` array with NCHW dimensions order. More details on blobs can be found [here](http://caffe.berkeleyvision.org/tutorial/net_layer_blob.html).

The following example illustrates the memory structure of a blob with N=2, C=16 channels and height H=5 / width W=4 ([source](https://intel.github.io/mkl-dnn/understanding_memory_formats.html)).

![](.section/software.png)

In a blob data structure, a value at position (n, c, h, w) can be accessed (if needed) using the following formula: b (n, c, h, w) = ((n C + c) H + h) * W + w.

The code below shows how an image loaded from the file is passed through the blobFromImage function to be converted into an input block for the neural network. The pixel values are scaled with a scaling factor of 1/255 to a target range of 0 to 1. It also adjusts the size of the image to the specified size of (416, 416, 416) without cropping.

```cpp
// generate 4D blob from input image
cv::Mat blob;
double scalefactor = 1/255.0;
cv::Size size = cv::Size(416, 416);
cv::Scalar mean = cv::Scalar(0,0,0);
bool swapRB = false;
bool crop = false;
cv::dnn::blobFromImage(img, blob, scalefactor, size, mean, swapRB, crop);
```

Later in the code, the output blob will be passed as input to the network. Then, a forward pass will be executed to obtain a list of predicted bounding boxes as output from the network. These boxes go through a post-processing step to filter out those with low confidence values. Let’s look at those steps in more detail.

### Step 4: Run Forward Pass Through the Network

As the next step, we have to pass the blob we just created to the network as its input. Then, we run the forward-function of OpenCV to perform a single forward-pass through the network. In order to do that, we need to identify the last layer of the network and provide the associated internal names to the function. This can be done by using the OpenCV function 'getUnconnectedOutLayers', which gives the names of all unconnected output layers, which are in fact the last layers of the network. The following code shows how this can be achieved:

```cpp
// Get names of output layers
vector<cv::String> names;
vector<int> outLayers = net.getUnconnectedOutLayers(); // get indices of output layers, i.e. layers with unconnected outputs
vector<cv::String> layersNames = net.getLayerNames(); // get names of all layers in the network

names.resize(outLayers.size());
for (size_t i = 0; i < outLayers.size(); ++i) // Get the names of the output layers in names
{
    names[i] = layersNames[outLayers[i] - 1];
}

// invoke forward propagation through network
vector<cv::Mat> netOutput;
net.setInput(blob);
net.forward(netOutput, names);
```

The result of the forward pass and thus the output of the network is a vector of size C (the number of blob classes) with the first four elements in each class representing the center in x, the center in y as well as the width and height of the associated bounding box. The fifth element represents the trust or confidence that the respective bounding box actually encloses an object. The remaining elements of the matrix are the confidence associated with each of the classes contained in the coco.cfg file. Further on in the code, each box is assigned to the class corresponding to the highest confidence.

The following code shows how to scan through the network results and assemble the bounding boxes with a sufficiently high confidence score into a vector. The function `cv::minMaxLoc` finds the minimum and maximum element values and their positions with extremums searched across the whole array.

```cpp
// Scan through all bounding boxes and keep only the ones with high confidence
float confThreshold = 0.20;
vector<int> classIds;
vector<float> confidences;
vector<cv::Rect> boxes;
for (size_t i = 0; i < netOutput.size(); ++i)
{
    float* data = (float*)netOutput[i].data;
    for (int j = 0; j < netOutput[i].rows; ++j, data += netOutput[i].cols)
    {
        cv::Mat scores = netOutput[i].row(j).colRange(5, netOutput[i].cols);
        cv::Point classId;
        double confidence;

        // Get the value and location of the maximum score
        cv::minMaxLoc(scores, 0, &confidence, 0, &classId);
        if (confidence > confThreshold)
        {
            cv::Rect box; int cx, cy;
            cx = (int)(data[0] * img.cols);
            cy = (int)(data[1] * img.rows);
            box.width = (int)(data[2] * img.cols);
            box.height = (int)(data[3] * img.rows);
            box.x = cx - box.width/2; // left
            box.y = cy - box.height/2; // top

            boxes.push_back(box);
            classIds.push_back(classId.x);
            confidences.push_back((float)confidence);
        }
    }
}
```

Applying the YOLOv3 algorithm to our highway image provides the following result:

![](.section/draggedimage-6.png)

As can be seen, the car in the left lane is covered by two bounding boxes of similar size. To avoid the occurrence of redundant boxes, the last step performs a non-maximum suppression which aims at keeping only the bounding box with the highest confidence score.

### Step 5: Post-Processing of Network Output

The OpenCV library offers a ready-made function for the suppression of overlapping bounding boxes. This function is called NMSBoxes and it can be used as illustrated by the following short code sample:

```cpp
// perform non-maxima suppression
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
vector<int> indices;
cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
```

After applying non-maximum suppression, redundant bounding boxes will have been successfully removed. The following figure shows the results, where green indicates preserved bounding boxes while red bounding boxes have been removed during NMS.

![](.section/draggedimage-8.png)

## Exercise

In the exercise (see [`detect_objects/`](detect_objects) subdirectory), we conduct the following experiments with the code given below:

1. Look at the `coco.cfg` file and find out which object classes the YOLO network is able to detect. Then, find some interesting images containing some of those objects and load them into the framework. Share some of the results with us if you like.
2. Experiment with the size of the blob image and use some other settings instead of 416 x 416. Measure the execution time for varying sizes of the blob image.
3. Experiment with the confidence threshold and the NMS threshold. How do the detection results change for different settings of both variables?

Note that we are now using a proprietary structure called `BoundingBoxes` to store the results of the object detection step. In the course of this lesson, further information will be added to this structure, such as Lidar points or image keypoints associated with the respective object.

![](.readme/result2.jpg)
