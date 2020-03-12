#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
bool matchFound = false;
float result[5];
int coordinateIndex = 0;


int ID = 5;

bool IDmatcher(int ID,int coordinateIndex)
{

    if (ID != -1)
    {

        //std::cout << " template_id is: " << ID << std::endl;
        if (ID == 0)
        {
            std::cout << "Raisin Bran" << std::endl;
            result[coordinateIndex] = ID;
        }
        else if (ID == 1)
        {
            std::cout << "Cinnamon Toast Crunch" << std::endl;
            result[coordinateIndex] = ID;
        }
        else if (ID == 2)
        {
            std::cout << "Rice Krispies " << std::endl;
            result[coordinateIndex] = ID;
        }

        else if (ID == 3)
        {
            std::cout << "Blank " << std::endl;
            result[coordinateIndex] = ID;
        }

        return true;
    }
    else
    {
        std::cout << " Match has not been found " << std::endl;
        return false;
    }
}
int main(int argc, char **argv)
{
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes;
    if (!boxes.load_coords() || !boxes.load_templates())
    {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for (int i = 0; i < boxes.coords.size(); ++i)
    {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: "
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // Execute strategy.
    while (ros::ok())
    {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        if (!matchFound)
        {
            int ID = imagePipeline.getTemplateID(boxes);
            // std::cout << " template_id is: " << ID << std::endl;

        if (IDmatcher(ID,coordinateIndex))
        {
            matchFound = true;
            std::cout << "ID MATCH FOUND: " << ID << std::endl;
            std::cout << "MOVE TO NEXT OBJECT" << std::endl;
        }
        }

        //  int ID = imagePipeline.getTemplateID(boxes);

       


        ros::Duration(0.01).sleep();
    }
    return 0;
}