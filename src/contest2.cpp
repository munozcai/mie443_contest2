#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

std::vector<float> get_box_offset(std::vector<float>  box, float offset){

    std::vector<float> offset_coords;

    float x = cos(box[2])*offset + box[0];
    float y = cos(box[2])*offset + box[1];

    offset_coords.push_back(x);
    offset_coords.push_back(y);
    offset_coords.push_back(box[2]);

    std::cout << "Box coordinates: " << std::endl;
        std::cout << " x: " << box[0] << " y: " << box[1] << " z: " 
                  << box[2] << std::endl;
     std::cout << "Target coordinates: " << std::endl;
        std::cout << " x: " << offset_coords[0] << " y: " << offset_coords[1] << " z: " 
                  << offset_coords[2] << std::endl;


    return offset_coords;
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }

    Boxes path; // reordered path to traverse. Include initial pos as destination

    path = boxes;

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.
    int state = 0;
    RobotPose initPos (0, 0, 0);
    bool move_done = false;
    int index = 4;

    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        if(state = 0){
            // save initial position
            initPos.x = robotPose.x;
            initPos.y = robotPose.y;
            initPos.phi = robotPose.phi;
            state = 1;
        }
        else if (state = 1){
            //test location
            if (!move_done){

                std::vector<float>  box = get_box_offset(boxes.coords[index], 0.7);

                move_done = Navigation::moveToGoal (box[0], box[1], box[2]);
            } else {
                ROS_INFO("Moving to next box");
                move_done = false;
                index -= 1;
                if (index == -1) index = 4;
            }
            
            // Go no next node
            // Image detection
            // Store tag and object location
            // Done traversing? Go back to initial position
        }http://wiki.ros.osuosl.org/costmap_2d
            
        


            
        
        
        //imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
