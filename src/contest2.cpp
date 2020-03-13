#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
bool matchFound = false;

typedef struct{
    std::string tag = "none";
    int tag_ID = -1;
    int coordinateIdx = -1;
    bool found = 0;
    bool repeated = 0;
} tag_info;

tag_info result [5];
int coordinateIndex = 4;

bool IDmatcher(int ID,int coord)
{

    if (ID != -1)
    {
        tag_info object_temp = result [coord];
        object_temp.tag_ID = ID;
        object_temp.coordinateIdx = coord;
        if (object_temp.found)
            object_temp.repeated = 1;
            
        else object_temp.found = 1;

        //std::cout << " template_id is: " << ID << std::endl;

        if (ID == 0) object_temp.tag = "Raisin Bran";
        else if (ID == 1) object_temp.tag = "Cinnamon Toast Crunch";
        else if (ID == 2) object_temp.tag = "Rice Krispies";
        else if (ID == 3) object_temp.tag = "Blank";

        std::cout << "Tag Matched for: " << object_temp.tag << std::endl;
        return true;
    }
    else
    {
        std::cout << " Match has not been found " << std::endl;
        //matchFound=false;
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

    Boxes path; // reordered path to traverse. Include initial pos as destination

    path = boxes;

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // Execute strategy.
<<<<<<< HEAD
    while (ros::ok())
    {
=======
    int state = 0;
    RobotPose initPos (0, 0, 0);

    while(ros::ok()) {
>>>>>>> lamc_branch
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
<<<<<<< HEAD
       // matchFound = false;
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

       


=======

        if(state = 0){
            // save initial position
            initPos.x = robotPose.x;
            initPos.y = robotPose.y;
            initPos.phi = robotPose.phi;
            state = 1;
        }
        else if (state = 1){
            //test location
            Navigation::moveToGoal (boxes.coords[0][0], boxes.coords[0][1], boxes.coords[0][2]);
            
            // Go no next node
            // Image detection
            // Store tag and object location
            // Done traversing? Go back to initial position
        }
            
        


            
        
        
        imagePipeline.getTemplateID(boxes);
>>>>>>> lamc_branch
        ros::Duration(0.01).sleep();
    }
    return 0;
}