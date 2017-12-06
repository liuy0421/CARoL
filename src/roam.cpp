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
#include <sensor_msgs/LaserScan.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string> 
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#define PI 3.14159265359
bool scan_heard = false;
float distanceThreshold = 1.0;
int nanThreshold = 10;
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_in);
const int DETECTION_THRESHOLD = 25;
const float WALL_DIST_THRESHOLD = 0.5;

// global variable that keeps scan info
sensor_msgs::LaserScan last_scan_in;
uint64_t scan_index = 0;


struct Coordinate {
    float x, y, z, w;
};

struct RoomInfo {
    int roomNum;
    std::string name;
    float priority;
    float initialPriority;
    bool visitSuccess = false;
    Coordinate* coords;
};



void sleepok(int t, ros::NodeHandle &nh) {
    if (nh.ok())
        sleep(t);
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
      
    //set relative x, y, and angle
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = z;
    goal.target_pose.pose.orientation.w = w;

    //send the goal
    ac.sendGoal(goal);
    
    //block until the action is completed
    ac.waitForResult();

    return 0;
}

void waitForScan(){		
	double elapsed_time = 0;
	int ratehz = 10;
	ros::Rate r(ratehz);

	scan_heard = false;

	while (ros::ok()){
		ros::spinOnce();
		r.sleep();
		elapsed_time += 1.0/(double)ratehz;
		if (scan_heard) 
			break;
	}
}


bool isOpen() {

	int realCount = 0;
	int longCount = 0;
	
	waitForScan();
	
	for (int i = last_scan_in.ranges.size() / 4; i < (3 * last_scan_in.ranges.size()) / 4; i++) {
		if (!isnan(last_scan_in.ranges[i])) {
			realCount++;
			if (last_scan_in.ranges[i] >= distanceThreshold) {
				longCount++;
			}	
		}
	}


	return (longCount >= realCount/ 2.0);

}

bool askToOpenDoor(ros::Publisher sound_pub, RoomInfo * roomInfo) {
    sound_play::SoundRequest S;
    
    S.sound = -3;
    S.command = 1;
    std::string messages;
    
    std::string cur_name = roomInfo->name;
    messages = std::string("Hello, ").append(cur_name.append(", please open the door."));
    S.arg = messages;
    sound_pub.publish(S);
    return true;
}

bool carol(ros::Publisher sound_pub, RoomInfo * roomInfo) {
    sound_play::SoundRequest S;
    S.sound = -2;
    S.command = 1;
    std::string messages;
    
    std::string cur_name = roomInfo->name;
	//char song_choice [1];
	//itoa(rand() % 3 + 1, song_choice, 10);
	std::stringstream stream;
    stream << rand() % 3 + 1;
	//std::string song_choice = std::to_string(rand() % 3 + 1);
    messages = std::string("/home/turtlebot/catkin_ws/src/CARoL/src/").append(stream.str().append(".mp3"));
    S.arg = messages;
    sound_pub.publish(S);
    for (int i = 0; i < 4; i++) {
		spin();
    }
    S.command = 0;
    sound_pub.publish(S);
    return true;
}

bool doDoor(ros::Publisher sound_pub, RoomInfo * roomInfo) {
	int countdown = 30 * 10;
	for (int i = 0; i < countdown; i++) {
		if (isOpen()) {
			roomInfo->visitSuccess=true;
			carol(sound_pub, roomInfo);
			roomInfo->priority = 0;
			return true;
		} else if (i % 100 == 0) {
			askToOpenDoor(sound_pub, roomInfo);			
		}
	}
	roomInfo->priority = roomInfo->priority / 2.0;
	return false;
}


void initRooms(std::vector<Coordinate *> *coordinates, std::vector<RoomInfo *> *roomInfo) {
    std::ifstream roomFile; 
    std::string pp = "/home/turtlebot/catkin_ws/src/CARoL/src/rooms/roomList.csv";
    roomFile.open(pp.c_str());
	int index = 0;
    if (roomFile.is_open()) {
        std::string line;
        char *ptr;
        getline(roomFile, line);    // getting rid of header
        while (getline(roomFile, line)) {
            int i = 6;
            RoomInfo *new_r = new RoomInfo;
            Coordinate *new_c = new Coordinate;
            const char *cstr = line.c_str();
            ptr = std::strtok ((char *)cstr, ",\n");
			index ++;

            while (ptr != NULL) {
                switch (i) {
                    case 6 :
                        new_r->roomNum = atoi(ptr);
                        break;
                    case 5 :
                        new_r->name = std::string(ptr);
                        break;
                    case 4 :
                        new_c->x = atof(ptr);
                        break;
                    case 3 :
                        new_c->y = atof(ptr);
                        break;
                    case 2 :
                        new_c->z = atof(ptr);
                        break;
                    case 1 :
                        new_c->w = atof(ptr);
                        break;
		    		case 0 :
						new_r->priority = atof(ptr);
						new_r->initialPriority = atof(ptr);
						break;
                }
                i--;
                ptr = strtok (NULL, ",\n");
            }
			new_r->coords = new_c;
			roomInfo->push_back(new_r);
			coordinates->push_back(new_c);
        }
    } else {
        std::cout << "Unable to open file :(\n";
    }
    roomFile.close();
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in) {
	last_scan_in = *scan_in;
	scan_index ++;
	scan_heard = true;
}


bool isDone(const std::vector<RoomInfo *> roomInfo) {
		
	for (uint32_t k = 0; k < 19; k++) {
		RoomInfo* info = roomInfo.at(k);
		if (info->priority >= 5) {
			return false;
		}	
	}
	return true;
}

RoomInfo *getNextRoom(const std::vector<RoomInfo *> roomInfo) {
	RoomInfo* max_room;
	float max_priority = 0;
	for (uint32_t k = 0; k < 19; k++) {
		RoomInfo* info = roomInfo.at(k);
		if (info->priority > max_priority) {
			max_priority = info->priority;
			max_room = info;
		}	
	}
	return max_room;
}

void writeRoomInfo(const std::vector<RoomInfo*> roomInfo) {
    	//std::string pp = "/home/turtlebot/catkin_ws/src/CARoL/src/rooms/roomList.csv";
    	std::ofstream f;
	f.open("/home/turtlebot/catkin_ws/src/CARoL/src/rooms/roomList.csv", std::ofstream::out);
	
	f << "room number,name, x, y, z, w, priority\n";
	for (uint32_t k = 0; k < 19; k++) {
		RoomInfo* info = roomInfo.at(k);
		float new_priority = info->initialPriority * ((info->visitSuccess) ? 2.0 : 0.5);
		f << info->roomNum;
		f << ", ";
		f << info->name;
		f << ", ";
		f << info->coords->x;
		f << ", ";
		f << info->coords->y;
		f << ", ";
		f << info->coords->z;
		f << ", ";
		f << info->coords->w;
		f << ", ";
		f << info->priority;
		f << "\n";	
	}
	f.close();
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
	ros::Subscriber laser_sub = n.subscribe("/scan", 1, laserCallback);

    
    //sleep for a bit to make sure the pub will work
    sleepok(2,n);
    

    double home_location[4] = {5.65, 13.8, 0.0, 0.0};
    
    int num_locations = 19;
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    // wait to make sure the service is there 
    // -- this has to be here even if you're use the service is already running
    ac.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
  
    double x1,y1,x2,y2,x3,y3,x4,y4 = 0;
    
    RoomInfo* nextTarget;
    while (ros::ok() && !isDone(roomInfo)) {

	nextTarget = getNextRoom(roomInfo);
	
        //move to next location
        Coordinate* coord = nextTarget->coords;
        move_turtle_bot(coord->x, coord->y, coord->z, coord->w);
        ROS_INFO("Moved to new location: " );
        
        ROS_INFO("speaking then sleeping..");
        doDoor(sound_pub, nextTarget);
    }
    writeRoomInfo(roomInfo);
     
    return 0;

}
