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

Mat img_object0;
Mat img_object1;
Mat img_object2;
Mat img_scene;
Mat descriptors_object0, descriptors_object1, descriptors_object2;
Mat descriptors_scene;

std::vector<KeyPoint> keypoints_object0, keypoints_object1, keypoints_object2;
std::vector<KeyPoint> keypoints_scene;

int minHessian = 400;
Ptr<SURF> detector = SURF::create(minHessian);
void descriptionInit();

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
//#define IMAGE_TOPIC "camera/image" // kinect:"camera/rgb/image_raw" webcam:"camera/image"
#define IMAGE_TOPIC "camera/image"

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

void ImagePipeline::imgObject(Boxes &boxes)
{
    img_object0 = boxes.templates[0]; // raisin
    img_object1 = boxes.templates[1];
    img_object2 = boxes.templates[2];
    descriptionInit();
}
int ImagePipeline::getTemplateID(Boxes &boxes)
{
    int template_id = -1;
    if (!isValid)
    {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    }
    else if (img.empty() || img.rows <= 0 || img.cols <= 0)
    {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    }
    else
    {
        /***YOUR CODE HERE***/
        // Use: boxes.templates
        std::cout << "VALID IMAGE!" << std::endl;
        // from template

        //from image feed
        img_scene = img;
        getID();

        //cv::imshow("view", img);
        cv::waitKey(10);
    }
    return template_id;
}

void descriptionInit()
{

    detector->detectAndCompute(img_object0, Mat(), keypoints_object0,
                               descriptors_object0);

    detector->detectAndCompute(img_object1, Mat(), keypoints_object1,
                               descriptors_object1);
    detector->detectAndCompute(img_object2, Mat(), keypoints_object2,
                               descriptors_object2);
}
int detection(Mat descriptors_object, Mat img_object, std::vector<KeyPoint> keypoints_object)
{
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;
    matcher.match(descriptors_object, descriptors_scene, matches);
    double max_dist = 0;
    double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < descriptors_object.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_object.rows; i++)
    {
        if (matches[i].distance < 3 * min_dist)
        {
            good_matches.push_back(matches[i]);
        }
    }

    if (good_matches.size() < 4)
    {
        std::cout << "Not enough good matches available in this image- Aldo" << std::endl;
    }
    else
    {
        Mat img_matches;
        if (keypoints_scene.size() < 1)
        {
            std::cerr << "ISSUE MAYBE\n";
            return -1 ;
        }
        drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for (int i = 0; i < good_matches.size(); i++)
        {
            //-- Get the keypoints from the good matches
            obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
            scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
        }

        std::cout << good_matches.size() << std::endl;
        Mat H = findHomography(obj, scene, RANSAC);
        
        if (H.empty())
        {
            return -1;
        }
        
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0, 0);
        obj_corners[1] = cvPoint(img_object.cols, 0);
        obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
        obj_corners[3] = cvPoint(0, img_object.rows);
        std::vector<Point2f> scene_corners(4);
        std::cout << "obj " << obj_corners.size() << std::endl;
        std::cout << "Scene " << scene_corners.size() << std::endl;
        std::cout << "H " << H << std::endl;
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
        //waitKey(0);
    }
}

int ImagePipeline::getID()
{
    if (!img_scene.data)
    {
       
        return -1;
    }

    detector->detectAndCompute(img_scene, Mat(), keypoints_scene,
                               descriptors_scene);

   int numGM = detection(descriptors_object2, img_object2, keypoints_object2);
    std::cout << " --(!) GM num "<< numGM << std::endl;

}
/** @function readme */
