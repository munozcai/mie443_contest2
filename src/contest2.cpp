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
float localized_coord[2];
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
        distance += sqr_dist(localized_coord[0], localized_coord[1], coord[2*(int(*a) - 49)], coord[2*(int(*(a+1)) - 49) + 1]);//x-> 2i, y-> 2i+1
        distance += sqr_dist(coord[2*(int(*(a+8)) - 49)], coord[2*(int(*(a+9)) - 49) + 1], localized_coord[0], localized_coord[1]);//for return distance

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


    
    //Optimized path code:
    float final_combination[10];
    float minDist=1000.;
    char coordinates_ID[] = "12345";
    int length = strlen(coordinates_ID);
    populate_dist(length);




    for(int i=0; i< boxes.coords.size(); i++)
    {
        int counter = 0;
        for(int j=0; j<2; j++){
            coord[counter] = boxes.coords[i][j];
            counter++;
        }

    }



    
    for(int i=0; i<length; i++){
        for(int j=0; j<length;j++){
            std::cout<<dist[i][j]<<"|";
        }
        std::cout<<std::endl;
    }


    std::cout << "Optimized Combination: " << combination<< "  min dist:  "<< minDist<< std::endl;


    int counter_1 = 0;
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
            localized_coord[0] = initPos.x;
            localized_coord[1] = initPos.y;
            permutation(coordinates_ID, 0, length, combination, minDist);

            for(int i=0;i<length;i++){//getting the coordinates for optimized route
                final_combination[counter_1] = coord[(int(combination[i]) - 49+1)];
                final_combination[counter_1 + 1] = coord[(int(combination[i]) - 49) +1+1];
                 std::cout << "Optimized Coordinates (index):   " << (int(combination[i]) - 49+1)<< " , "<< (int(combination[i]) - 49) +1+1 << std::endl;
                counter_1 += 2;
            }    
                std::cout << "Optimized Combination: " << combination<< "  min dist:  "<< minDist<< std::endl;
            
            for(int i=0; i< 0.5*(sizeof(final_combination)/sizeof(int)) ;i++){
                std::cout << "Optimized Coordinates:   " << final_combination[i]<< "  ,  "<< final_combination[i+1] << std::endl;
            }         
            
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
