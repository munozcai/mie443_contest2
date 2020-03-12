#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <iostream>
#include <bits/stdc++.h>
#include <string>

//float distance, minDist;
char combination[10];
float coord[10];//={8.5,1.0,  1.2,1.5,  1.3,1.5, -1.2,-2.3,  1.3,3.7};//C1, C2, C3, C4, C5

//to fill out coordinates x, y with 1 followed by 2, 3, 4, 5

float dist[10][10];//distance matrix ABCD X ABCD for coordinates ABCD;
float sqr_dist(float x1, float y1, float x2, float y2){
    return std::sqrt(  ((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)) );
}
void populate_dist(int n){
    int p=0;
    int q=0;
    for(int i=0; i<n; i++){
        for(int j=0; j<n;j++){
            dist[i][j] = sqr_dist(coord[p], coord[p+1], coord[q], coord[q+1]);
            q=q+2;
        }
        p=p+2;
        q=0;
    }
}


void swap_fcn(char *a, char *b){
    char temp = *a;
    *a = *b;
    *b = temp;
}

void permutation(char *a, int l, int r, char *b, float &minDist){//l-> start of string, r-> end of string ABCD, a-> ptr to first character of string
    float distance;
    //float minDist = distance;

    if(l==r){
        distance=0.;
        for(int k=0;k<r;k++){
            //distance = dist[(int(*a) - 65)][(int(*(a+1)) - 65)] + dist[(int(*(a+1)) - 65)][(int(*(a+2)) - 65)] + dist[(int(*(a+2)) - 65)][(int(*(a+4)) - 65)];
            distance += dist[(int(*(a+k)) - 49)][(int(*(a+k+1)) - 49)];
        }
        if(distance < minDist && distance>0.){
            for(int p=0;p<r;p++){
                *(b+p) = *(a+p);
            }
            minDist=distance;
        }
        //cout<<a<<endl;
        std::cout<<"string: "<<a<<" Dist: " <<distance<< "  min dist: "<< minDist <<std::endl;

    }

    else{
        for(int i = l; i < r; i++){
            swap_fcn((a+l), (a+i));
            permutation(a, l+1, r, combination, minDist);//recursive call
            swap_fcn((a+l), (a+i));//change it back to original for the next for loop
        }
    }
}


std::vector<float> get_box_offset(std::vector<float>  box, float offset){

    std::vector<float> offset_coords;

    float phi = box[2];
    float x = cos(phi)*offset + box[0];
    float y = sin(phi)*offset + box[1];
    

    offset_coords.push_back(x);
    offset_coords.push_back(y);
    offset_coords.push_back(phi+M_PI);

    std::cout << "Box coordinates: " << std::endl;
        std::cout << " x: " << box[0] << " y: " << box[1] << " z: " 
                  << box[2] << std::endl;
     std::cout << "Target coordinates: " << std::endl;
        std::cout << " x: " << offset_coords[0] << " y: " << offset_coords[1] << " z: " 
                  << offset_coords[2] << std::endl;


    return offset_coords;
}

typedef enum  {
    NONE = 0,
    INITIALIZE,
    PATH_PLANNER,
    MOVE_TO_TARGET,
    CAPTURE,
    RETRY_TARGET,
    DONE,
} STATE;

#define TARGET_OFFSET 0.6

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

  

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.
    STATE state = INITIALIZE;
    RobotPose initPos (0, 0, 0);
    bool move_done = false;
    int index_target = 0;


    
    //Optimized path code:

    for(int i=0; i< boxes.coords.size(); i++)
    {
        int counter = 0;
        for(int j=0; j<2; j++){
            coord[counter] = boxes.coords[i][j];
            counter++;
        }

    }



    float minDist=1000.;
    char coordinates_ID[] = "12345";
    int length = strlen(coordinates_ID);
    populate_dist(length);

    for(int i=0; i<length; i++){
        for(int j=0; j<length;j++){
            std::cout<<dist[i][j]<<"|";
        }
        std::cout<<std::endl;
    }
    permutation(coordinates_ID, 0, length, combination, minDist);

    std::cout << "Optimized Combination: " << combination<< "  min dist:  "<< minDist<< std::endl;

    std::vector<std::vector<float>> path; // reordered path to traverse. Include initial pos as destination

    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        switch (state)
        {
        case INITIALIZE:
            // Localize initial position
            ros::Duration(5).sleep();
            ros::spinOnce();
            
            // Save initial position
            initPos.x = robotPose.x;
            initPos.y = robotPose.y;
            initPos.phi = robotPose.phi;

            // calculate target coordinates
            for(int i = 0; i < boxes.coords.size(); ++i) {
                std::cout << "Box coordinates: "  << i << std::endl;
                path.push_back(get_box_offset(boxes.coords[i], TARGET_OFFSET));
            }

            state = PATH_PLANNER;
            break;

        case PATH_PLANNER: // this is state is used again later to recalculate best path among unreachable targets
            // Find optimal path

            // ADD TSP HERE
            // TSP(initPos, path); // modify path to reflect ordered list


            state = MOVE_TO_TARGET;
            break;
            
        case MOVE_TO_TARGET:

            if ( path.empty()){
                state = DONE;
                break;
            }
            // Move base to target
            move_done = Navigation::moveToGoal (path[index_target][0], path[index_target][1], path[index_target][2]);

            // Navigation done
            if (move_done){
                ROS_INFO("Target reached! %d left", path.size()-1);
                index_target = 0; // first element is next
                path.erase(path.begin()); // delete entry from list
                state = CAPTURE;
            } 
            // Navigation failed, go to next target
            else { 
                if (index_target < path.size() - 1 ){
                    ROS_ERROR("UNREACHABLE TARGET, going to next target");
                    index_target += 1;
                }
                else { // retry from first element 
                    index_target = 0; // restart 
                    state = RETRY_TARGET;
                }
            }
            break;

        case CAPTURE:
            // ADD OPEN CV CODE HERE

            state = RETRY_TARGET;
            break;

        case RETRY_TARGET:
        
            // Replan path wigh targets left
            state = PATH_PLANNER;
            break;

        case DONE:
            
            // Go back to initial position forever
            Navigation::moveToGoal (initPos.x, initPos.y, initPos.phi);

            break;

        default:
            break;
        }

       
        


            
        
        
        //imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
