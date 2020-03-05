#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

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
            Navigation::moveToGoal (boxes.coords[0][0], boxes.coords[0][1], boxes.coords[0][2]);
            
            // Go no next node
            // Image detection
            // Store tag and object location
            // Done traversing? Go back to initial position
        }
            
        


            
        
        
        imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
