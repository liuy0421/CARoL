/* modified from the Halloween robot source code 
   provided by jsinapov */

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

#include <sound_play/sound_play.h>
#include <sound_play/SoundRequest.h>

#include <vector>
#include <iostream>
#include <fstream>

#define PI 3.14159265359


struct RoomInfo {
    int roomNum;
    std::string name;
};

struct Coordinate {
    float x, y, z, w;
};

void sleepok(int t, ros::NodeHandle &nh) {
    if (nh.ok())
        sleep(t);
}

void shake(int times) {
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    //wait to make sure the service is there 
    //-- this has to be here even if you're use the service is already running
    ac.waitForServer(); 
	move_base_msgs::MoveBaseGoal goal;

	for (int i = 0; i < times * 2; i++) {
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.header.frame_id = "/base_link";
		
		int dir = ((i % 2) * 2) - 1;
		
		//set relative x, y, and angle
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(40 * dir);

		//send the goal
		ac.sendGoal(goal);
		
		//block until the action is completed
		ac.waitForResult();
	}
}

void spin() {
	move_base_msgs::MoveBaseGoal goal;
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    //wait to make sure the service is there 
    //-- this has to be here even if you're use the service is already running
    ac.waitForServer(); 

	for (int i = 0; i < 3; i++) {
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.header.frame_id = "/base_link";
		
		//set relative x, y, and angle
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(120 * i);

		//send the goal
		ac.sendGoal(goal);
		
		//block until the action is completed
		ac.waitForResult();
	}	
}

void dance() {
    shake(4);
    spin();
    shake(4);
}

int move_turtle_bot (double x, double y, double z, double w) {
	std::cout << "In move_turtle_bot" << std::endl;
    // changed to handle orientation z and w  

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    //wait to make sure the service is there 
    //-- this has to be here even if you're use the service is already running
    ac.waitForServer(); 
    move_base_msgs::MoveBaseGoal goal;
    
    std::cout<< "Going to : " << x  << ", "<< y << std::endl;
    
    //set the header
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/map"; 
    //     goal.target_pose.header.frame_id = "/base_link";
      
    //set relative x, y, and angle
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = z;
    goal.target_pose.pose.orientation.w = w;

    //send the goal
    ac.sendGoal(goal);
    
    //block until the action is completed
    ac.waitForResult();
    //~ std::cout<<"\n \n waiting for " << sleep_time<<" seconds \n \n";
    //~ sleep(sleep_time);
  
  return 0;
}

void sayRandomPhrase(ros::Publisher sound_pub, int index, const std::vector<RoomInfo *> &roomInfo){
    sound_play::SoundRequest S;
    
    S.sound = -3;
    S.command = 1;
    
    //int num_messages = 3;
    
    //int m = rand() % num_messages;
    std::string cur_name = roomInfo.at(index)->name;
    
    std::string messages = std::string("Hello, ").append(cur_name.append(", open the door."));
    
    S.arg = messages;
    
    sound_pub.publish(S);
}

void initRooms(std::vector<Coordinate *> *coordinates, std::vector<RoomInfo *> *roomInfo) {
    std::ifstream roomFile; 
    std::string pp = "/home/turtlebot/catkin_ws/src/carol_exec/src/rooms/roomList.csv";
    //char *roomPath = pp.c_str();
    roomFile.open(pp.c_str());
	int index = 0;
    if (roomFile.is_open()) {
        std::string line;
        char *ptr;
        getline(roomFile, line);    // getting rid of header
        while (getline(roomFile, line)) {
            int i = 5;
            RoomInfo *new_r = new RoomInfo;
            Coordinate *new_c = new Coordinate;
            const char *cstr = line.c_str();
            ptr = std::strtok ((char *)cstr, ",\n");
			index ++;

            while (ptr != NULL) {
                switch (i) {
                    case 5 :
						std::cout << "index is : " << index << " room is " << atoi(ptr) << std::endl;
                        new_r->roomNum = atoi(ptr);
                        break;
                    case 4 :
                        new_r->name = std::string(ptr);
                        break;
                    case 3 :
                        new_c->x = atof(ptr);
                        break;
                    case 2 :
                        new_c->y = atof(ptr);
                        break;
                    case 1 :
                        new_c->z = atof(ptr);
                        break;
                    case 0 :
                        new_c->w = atof(ptr);
                        break;
                }


                i--;
                ptr = strtok (NULL, ",\n");
            }
			roomInfo->push_back(new_r);
			coordinates->push_back(new_c);
        }
    } else {
        std::cout << "Unable to open file!!!!!\n";
    }

    roomFile.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;
    std::vector<Coordinate *> coordinates;
    std::vector<RoomInfo *> roomInfo;

    initRooms(&coordinates, &roomInfo);
    //publisher for sound
    ros::Publisher sound_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 1);
    
    
    //sleep for a bit to make sure the pub will work
    sleepok(2,n);
    

    double home_location[4] = {5.65, 13.8, 0.0, 0.0};
    
    int num_locations = 19;
    // double locations[19][4] = { {2.77, 5.86, -0.82, 0.57},  // 202
    //                             {7.14, 5.87, -0.82, 0.56},  // 205
    //                             {10.71, 6.58, 0.79, 0.62},  // 206
    //                             {12.18, 5.56, -0.8, 0.6},   // 207
    //                             {13.36, 5.55, -0.62, 0.78}, // 208
    //                             {17.4, 5.56, -0.84, 0.54},  // 210
    //                             {19.02, 5.4, -0.8, 0.6},    // 211
    //                             {23.36, 5.4, -0.54, 0.84},  // 213
    //                             {24.47, 5.4, -0.64, 0.77},  // 214
    //                             {26.16, 6.23, 0.06, 1.0},   // 215
    //                             {25.68, 14.41, 0.64, 0.77}, // 220
    //                             {24.16, 14.39, 0.59, 0.81}, // 221
    //                             {19.55, 14.39, 0.84, 0.55}, // 222
    //                             {17.96, 14.22, 0.61, 0.8},  // 224
    //                             {-3.15, 14.03, 0.78, 0.62}, // 235a
    //                             {-4.55, 14.23, 0.78, 0.63}, // 237
    //                             {-7.63, 15.23, 0.7, 0.72},  // 239
    //                             {-7.72, 13.71, 1, 0.06},    // 241
    //                             {2.45, 13.39, -0.8, 0.61},  // 250
    //                             };
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    // wait to make sure the service is there 
    // -- this has to be here even if you're use the service is already running
    ac.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
  
    double x1,y1,x2,y2,x3,y3,x4,y4 = 0;
    
    int c = 0;  
    
    // testing
    for (uint32_t k = 0; k < 19; k++) {
		RoomInfo* info = roomInfo.at(k);
		std::cout << k << " : " << info->roomNum << std::endl;

	}
    
    
    while (ros::ok()) {
		c = rand() % 19;
		std::cout << "looping ... " << c <<  std::endl;
        //move to next location
        Coordinate* coord = coordinates.at(c);
        dance();
        //~ move_turtle_bot(coord->x, coord->y, coord->z, coord->w);
        ROS_INFO("Moved to new location: " );
        std::cout << coord->x << std::endl;
        std::cout << coord->y << std::endl;
        std::cout << coord->z << std::endl;
        std::cout << coord->w << std::endl;
        
        //say something I'm giving up on you
        //sc.say("Hello!");
        ROS_INFO("speaking then sleeping..");
        
        /* What does this part do? */
        //for (int p = 0; p < 3; p ++){
             sayRandomPhrase(sound_pub, c, roomInfo);
            // sleepok(4,n);
        //     move_turtle_bot(locations[c][0],locations[c][1],locations[c][2], locations[c][3]);
        //}
        sleepok(10,n);
        
        //increment location 

        //c = (c + 1) % num_locations;
        // c++;
        
        // if (c >= num_locations){
        //  c = 0;
        // }
    }
     
    return 0;

}
