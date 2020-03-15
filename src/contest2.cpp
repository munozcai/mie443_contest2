#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <iostream>
#include <bits/stdc++.h>
#include <string>


/* *************************************************************** 
                START Helper Functions
 *************************************************************** */
//float distance, minDist;
char combination[10];
float coord[10];//={8.5,1.0,  1.2,1.5,  1.3,1.5, -1.2,-2.3,  1.3,3.7};//C1, C2, C3, C4, C5
float localized_coord[2];
int ID;
//to fill out coordinates x, y with 1 followed by 2, 3, 4, 5

// Distance matrix ABCD X ABCD for coordinates ABCD. Takes two end points (x,y) and returns their distance
float dist[10][10]; 
float sqr_dist(float x1, float y1, float x2, float y2)
    return std::sqrt(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)));
{
}
void populate_dist(int n)
{
    int p = 0;
    int q = 0;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            dist[i][j] = sqr_dist(coord[p], coord[p + 1], coord[q], coord[q + 1]);
            q = q + 2;
        }
        p = p + 2;
        q = 0;
    }
}

void swap_fcn(char *a, char *b)
{
    char temp = *a;
    *a = *b;
    *b = temp;
}

void permutation(char *a, int l, int r, char *b, float &minDist)
{ //l-> start of string, r-> end of string ABCD, a-> ptr to first character of string
    float distance;
    //float minDist = distance;

    if (l == r)
    {
        distance = 0.;
        for (int k = 0; k < r; k++)
        {
            //distance = dist[(int(*a) - 65)][(int(*(a+1)) - 65)] + dist[(int(*(a+1)) - 65)][(int(*(a+2)) - 65)] + dist[(int(*(a+2)) - 65)][(int(*(a+4)) - 65)];
            distance += dist[(int(*(a + k)) - 49)][(int(*(a + k + 1)) - 49)];
        }
        distance += sqr_dist(localized_coord[0], localized_coord[1], coord[2*(int(*a) - 49)], coord[2*(int(*(a+1)) - 49) + 1]);//x-> 2i, y-> 2i+1
        distance += sqr_dist(coord[2*(int(*(a+8)) - 49)], coord[2*(int(*(a+9)) - 49) + 1], localized_coord[0], localized_coord[1]);//for return distance
        if(distance < minDist && distance>0.){
            for(int p=0;p<r;p++){
                *(b+p) = *(a+p);

            }
            minDist = distance;
        }
        //cout<<a<<endl;
        std::cout << "string: " << a << " Dist: " << distance << "  min dist: " << minDist << std::endl;
    }

    else
    {
        for (int i = l; i < r; i++)
        {
            swap_fcn((a + l), (a + i));
            permutation(a, l + 1, r, combination, minDist); //recursive call
            swap_fcn((a + l), (a + i));                     //change it back to original for the next for loop
        }
    }
}

// Calcualtes target location at a TARGET_OFFSET distance from box center facing the object
#define TARGET_OFFSET 0.4
std::vector<float> get_box_offset(std::vector<float> box, float offset)
{
    std::vector<float> offset_coords;

    float phi = box[2];
    float x = cos(phi) * offset + box[0];
    float y = sin(phi) * offset + box[1];

    offset_coords.push_back(x);
    offset_coords.push_back(y);
    offset_coords.push_back(phi + M_PI);

    std::cout << "Box coordinates: " << std::endl;
    std::cout << " x: " << box[0] << " y: " << box[1] << " z: "
              << box[2] << std::endl;
    std::cout << "Target coordinates: " << std::endl;
    std::cout << " x: " << offset_coords[0] << " y: " << offset_coords[1] << " z: "
              << offset_coords[2] << std::endl;

    return offset_coords;
}
// IDmatcher() to be called after image has been detected and an ID was found
// This function initializes the array <tag_info> result for reporting purposes of identified tags and object locations
typedef struct
{
    std::string tag = "NOT SET";
    int tag_ID = -10;
    int coordinateIdx = -1;
    bool repeated = 0;
} tag_info;

tag_info result[5];
int coordinateIndex = 4;

bool id_matcher(int ID, int coordinates)
{

    if (ID != -1)
    {
        tag_info object_temp;
        object_temp.tag_ID = ID;
        object_temp.coordinateIdx = coordinates;
        
        for (int i = 0; i < 5; i++){
            if (ID == result[i].tag_ID) object_temp.repeated = 1;
        }

        //std::cout << " template_id is: " << ID << std::endl;

        if (ID == 0)
            object_temp.tag = "Raisin Bran";
        else if (ID == 1)
            object_temp.tag = "Cinnamon Toast Crunch";
        else if (ID == 2)
            object_temp.tag = "Rice Krispies";
        else if (ID == 3)
            object_temp.tag = "Blank";

        std::cout << "Tag Matched for: " << object_temp.tag << "ID: "<< ID << std::endl;
        result[coordinates] = object_temp;
        return true;
    }
    else
    {
        std::cout << " Match has not been found " << std::endl;
        //matchFound=false;
        return false;
    }
}


// gen_txt() is called when the robot has finished traversing or when the timer expired.
// this funtion creates a .txt file with the tags and the objects location
void gen_txt(Boxes finalPath)
{
    std::cout << "\n Generating report files... \n";
    const char *path = "/home/file.txt";
    std::ofstream file(path);
    std::string data = "********* MIE443 -Contest 2 *********\nTeam 15 \n \n OBJECT AND TAG REPORT:";

    for (int i = 0; i < finalPath.coords.size(); i++)
    {
        tag_info object_temp = result[i];

        std::string ID = "\n \n Node ID:\t" + std::to_string(object_temp.tag_ID);
        std::string tag = "\n Tag:\t \t" + object_temp.tag;
        data += ID + tag;
        if (object_temp.coordinateIdx != -1)
        {
            std::string coord = "\n Coordinates: \n \t x: \t" + std::to_string(finalPath.coords[object_temp.coordinateIdx][0]) 
            + "\t y: \t" + std::to_string(finalPath.coords[object_temp.coordinateIdx][1]) 
            + "\t phi: \t" + std::to_string(finalPath.coords[object_temp.coordinateIdx][2]);
            data += coord;
        }
        else
            data += "\n Coordinates: \tNo object set";

        if (object_temp.repeated)
            data += "\n THIS IS A REPEATED TAG!\n";
    }

    file << data;

    file.close();
    std::cout << data;
}
/* *************************************************************** 
                END Helper Functions
 *************************************************************** */

typedef enum
{
    NONE = 0,
    INITIALIZE,
    PATH_PLANNER,
    MOVE_TO_TARGET,
    CAPTURE,
    RETRY_TARGET,
    DONE,
} STATE;

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
    STATE state = INITIALIZE;
    RobotPose initPos(0, 0, 0);
    bool move_done = false;
    int index_target = 0;
    std::vector<std::vector<float>> path; // reordered path to traverse. Include initial pos as destination
    std::vector<float> path_idx;
    //Optimized path code:
    float final_combination[10];
    float minDist=1000.;
    char coordinates_ID[] = "12345";
    int length = strlen(coordinates_ID);
    populate_dist(length);
    for(int i=0; i< boxes.coords.size(); i++)
    {
        int counter = 0;
        for (int j = 0; j < 2; j++)
        {
            coord[counter] = boxes.coords[i][j];
            counter++;
        }
    }

    float minDist = 1000.;
    char coordinates_ID[] = "12345";
    int length = strlen(coordinates_ID);
    populate_dist(length);

    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < length; j++)
        {
            std::cout << dist[i][j] << "|";
        }
        std::cout << std::endl;
    }

    std::cout << "Optimized Combination: " << combination<< "  min dist:  "<< minDist<< std::endl;

    while (ros::ok() && !all_done) // add timer of 5 minutes
    {
    int counter_1 = 0;
    bool all_done = false;
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        switch (state)
        {
        case INITIALIZE:
            // Localize initial position
            std::cout << "\n NOTE: Ensure that robot is localized \n";
            ros::Duration(5).sleep();
            ros::spinOnce();

            // Save initial position. For "returning to initial position" purposes
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

            std::cout << "Initial Position: " << std::endl;
            std::cout << " x: " << initPos.x << " y: " << initPos.y << " z: " << initPos.phi << std::endl;

            // Calculate target coordinates with an OFFSET
            for (int i = 0; i < boxes.coords.size(); ++i)
            {
                std::cout << "Box coordinates: " << i << std::endl;
                path.push_back(get_box_offset(boxes.coords[i], TARGET_OFFSET));
                path_idx.push_back(i);
            }

            state = PATH_PLANNER;
            break;

        case PATH_PLANNER: // this is state is used again later to recalculate best path among unreachable targets
            // Find optimal path

            // ADD TSP HERE
            // TSP(initPos, path); // modify path to reflect ordered list
            // take care of path_idx 

            state = MOVE_TO_TARGET;
            break;

        case MOVE_TO_TARGET:

            if (path.empty())
            {
                state = DONE;
                break;
            }
            // Move base to target
            move_done = Navigation::moveToGoal(path[index_target][0], path[index_target][1], path[index_target][2]);

            // Navigation done
            if (move_done)
            {
                ROS_INFO("Target reached! %d targets left", path.size() - 1);

                state = CAPTURE;
            }
            // Navigation failed, go to next target
            else
            {
                if (index_target < path.size() - 1)
                {
                    ROS_ERROR("UNREACHABLE TARGET, going to next target");
                    index_target += 1;
                    state = MOVE_TO_TARGET;
                    break;
                }
                else
                { // retry from first element
                    state = RETRY_TARGET;
                }
            }
            break;

        case CAPTURE:
            // ADD OPEN CV CODE HERE

            ID = imagePipeline.getTemplateID(boxes);

            // std::cout << " template_id is: " << ID << std::endl;

            if (id_matcher(ID, int(path_idx[index_target])))
            {
                // matchFound = true;
                std::cout << "ID MATCH FOUND: " << ID << std::endl;
                std::cout << "MOVE TO NEXT OBJECT" << std::endl;

                path.erase(path.begin() + index_target); // delete entry from list
                path_idx.erase(path_idx.begin() + index_target);

                state = MOVE_TO_TARGET;
                break;
            }
            else
            {
                index_target += 1;
                state = RETRY_TARGET;
                break;
            }

        case RETRY_TARGET:

            // Replan path wigh targets left
            // could add more fancy staff such as timers etc...
            state = PATH_PLANNER;
            break;

        case DONE:

            // Go back to initial position forever
            all_done = Navigation::moveToGoal(initPos.x, initPos.y, initPos.phi);

            break;

        default:
            break;
        }
        
        //imagePipeline.getTemplateID(boxes);
        // matchFound = false;

        //  int ID = imagePipeline.getTemplateID(boxes);

        ros::Duration(0.01).sleep();
    }

    gen_txt(boxes);

    return 0;
}