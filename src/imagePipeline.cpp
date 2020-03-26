#include <imagePipeline.h>
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"
using namespace cv;
using namespace cv::xfeatures2d;

using namespace cv;
using namespace cv::xfeatures2d;
#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "/camera/rgb/image_raw"// kinect:"camera/rgb/image_raw" webcam:"camera/image"

int minHessian = 400;
Ptr<SURF> detector = SURF::create(minHessian);
Mat descriptors_object, descriptors_scene;
Mat img_object;
Mat img_scene;
Mat img;
Mat H;
int maxGM = 0;
int currentGM;
int template_id = 3;
std::vector<KeyPoint> keypoints_object;
std::vector<KeyPoint> keypoints_scene;
std::vector<Point2f> obj;
std::vector<Point2f> scene;
std::vector<std::vector<DMatch>> knn_matches;
std::vector<DMatch> good_matches;
std::vector<Point2f> obj_corners(4);
std::vector<Point2f> scene_corners(4);
int templateIDArray[3] = {1, 1, 1};

ImagePipeline::ImagePipeline(ros::NodeHandle &n)
{
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        if (isValid)
        {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}

int detectGM(int templateID)
{
    knn_matches.clear();
    good_matches.clear();
    obj.clear();
    scene.clear();
    keypoints_object.clear();
    keypoints_scene.clear();
    Mat img_matches;
    obj_corners.clear();
    scene_corners.clear();
    // H.clear();

    detector->detectAndCompute(img_object, Mat(), keypoints_object,
                               descriptors_object);
    detector->detectAndCompute(img_scene, Mat(), keypoints_scene,
                               descriptors_scene);

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

    matcher->knnMatch(descriptors_object, descriptors_scene, knn_matches, 2);

    const float ratio_thresh = 0.75f;

    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    if (keypoints_scene.size() < 1)
    {
        std::cerr << "ISSUE MAYBE\n";
        return -1;
    }

    if (good_matches.size() < 4)
    {
        std::cout << "Not enough good matches available in this image- Aldo" << std::endl;
        //templateIDArray[templateID] = 0;
        return -1;
    }
    else
    {
        drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        //-- Localize the object
        //obj.clear();
        //scene.clear();
        std::vector<Point2f> obj_corners(4);
        std::vector<Point2f> scene_corners(4);

        for (int i = 0; i < good_matches.size(); i++)
        {
            //-- Get the keypoints from the good matches
            obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
            scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
        }

        //std::cout << good_matches.size() << std::endl;

        Mat H = findHomography(obj, scene, RANSAC);

        if (H.empty())
        {
            return -1;
        }

        //-- Get the corners from the image_1 ( the object to be "detected" )

        obj_corners[0] = cvPoint(0, 0);
        obj_corners[1] = cvPoint(img_object.cols, 0);
        obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
        obj_corners[3] = cvPoint(0, img_object.rows);

        //debug
        //std::cout << "obj " << obj_corners.size() << std::endl;
        //std::cout << "Scene " << scene_corners.size() << std::endl;
        //std::cout << "H " << H << std::endl;
        
        perspectiveTransform(obj_corners, scene_corners, H);

        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0),
             scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0),
             scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0),
             scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0),
             scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);

        //-- Show detected matches
        imshow("Good Matches & Object detection", img_matches);

        return good_matches.size();

        waitKey(0);
    }
}

int ImagePipeline::getTemplateID(Boxes &boxes)
{
   // cv::waitKey(800);
    img_scene = img;

    if (!isValid)
    {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
        return -1;
    }
    else if (img.empty() || img.rows <= 0 || img.cols <= 0)
    {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
        return -1;
    }
    else
    {
        /***YOUR CODE HERE***/
        // Use: boxes.templates
        cv::waitKey(200);
        template_id = 4;
       
        for (int i = 0; i < 3; i++)
        {

            img_object = boxes.templates[i];

            if (!img_object.data || !img_scene.data)
            {
                std::cout << " --(!) Error reading images " << std::endl;
                return -1;
            }
            else
            {
               
                currentGM = detectGM(i);

                //std::cout << " Number of Good Matches is: "<< currentGM << std::endl;
                std::cout << " Number of Good Matches is: " << currentGM << std::endl;

                if (maxGM < currentGM)
                {
                    maxGM = currentGM;
                    template_id = i;
                    std::cout << " Max Good Matches is: " << maxGM << std::endl;
                }
            }
        }

        //cv::imshow("view", img);

        cv::waitKey(10);
    }

    if (maxGM > 55)
    {
        return template_id;
    }
    else
    {
        return 3;
    }
}
