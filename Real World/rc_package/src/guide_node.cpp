#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h> //LIBRARY TO RUN AC COMMANDS (POSITION GOALS) TO MOVE_BASE

#include "geometry_msgs/Twist.h" // MOTOR COMMANDS
#include "sensor_msgs/LaserScan.h" // LASER DATA
#include "tf/transform_listener.h" // TF TREE

#include <map>
#include <fstream>
#include <string>

// DETECTION MSGS
#include <pal_detection_msgs/Detections2d.h>

#include <std_msgs/String.h>

//--------------------------------------DECLARATION OF FLAGS AND VARIABLES----------------------------------------------//

//STRUCT OF POINT READ ON THE QRCODES TO PASS TO THE ALGORITHM (FWY)
struct Point{
    float x;
    float y;
	double yaw;
	int label;
};

//SECTIONS TO DIFFERENCIATE READS FROM LASER SENSOR
struct Section
{
  float front;
  float left;
  float right;
} section;


Point points_database[5]; //STORES THE POINTS READ ON THE QRCODES

int a = 0; //CONTER TO MOVEMENTS MADE TO DEFINE STOPS BETWEENS INTANCES

bool detected = 0; //FLAG TO SAY IF THERE WAS DETECTED A PERSON WHILE DESINFECTING (USED ON FWY)
bool barcode = 0; //FLAG TO SAY IF THE QRCODE WAS READ
bool desinfecao = 0; //FLAG TO SAY IF THE ROOM IS BEING DESINFECTED
bool save_detected = 0; //FLAG TO TRY ERRADICATE CAMERA ERRORS ON DETECTING PERSONS

//VARIABLES THAT SAVES REAL TIME POSE OF THE ROBOT
float robot_pose_x = 0;
float robot_pose_y = 0;
float robot_pose_z = 0;
float robot_pose_w = 0;

int room_flag = 0; //COUNTER THAT SAVES THE ROOM THAT WILL BE DESINFECTED
float found = 0; //FLAG THAT SAYS IF ON INTEGER WAS FOUND ON A STRING READ ON THE QRCODE (FW)


bool fw_running = 0; //FLAG THAT SAYS IF THE FW MOVEMENT IS RUNNING, TO RESTRING OTHERS THINGS IN THE ALGORITHM
int fw_room = 0; //FLAG THAT SAVES THE ROOM THAT WAS DESINFECTED ON THE FW ALGORITHM

//FLAGS THAT SAVES THE ALGORITHM READ ON THE QRCODE
bool fw_flag = 0;
bool fwy_flag = 0;
bool dr_flag = 0;

int count_lines = 0; //COUNTS THE LINES OF POINTS ON THE QRCODE (FWY) - AVOID SEGMENTATION FAULT ON READING WRONG NUMBER OF POINTS
int follow_dir = -1; //DIRECTION OF THE MOVEMENT ON FW ALGORITHM
int state_ = 0; //STATES OF MOVEMTS ON THE FW ALGORITHM

ros::Publisher motor_command_publisher; //PUBLISHER TO SEND VELOCITIES TO THE ROBOT - USED ON THE MOVEMENT TO READ QRCODES AND FW
geometry_msgs::Twist velocity; //TYPE OF MESSAGE TO SEND VELOCITIES TO PUBLISHER


//--------------------------------------CALLBACK FUNCTIONS----------------------------------------------//

//THIS FUNCTION IS TO EXTRACT INTEGERS OF STRINGS. IT IS USED TO READ THE FOLLOW WALL MODE FROM QRCODE
bool extractIntegerWords(std::string str)
{
	std::string str1;
	std::string str2;
	int n = 0;
	//float found = 0;

	for (int x = 0; x < str.length(); x++)
	{
		if (isdigit(str[x]) && n == 0)
		{
			str1 = str[x];
			n++;
		}
		else if(isdigit(str[x]) && n == 1)
		{
			str2 = str[x];
			std::string valor = str1 + "." + str2;
			found = stof(valor);
			return 1;
		}
	}
	return 0;
}

//---CALLBACK TO RECEIVE THE ACTUAL POSE OF THE ROBOT IN REAL-TIME---//
void callback_pose(const geometry_msgs::Pose& msg)
{
	robot_pose_x = msg.position.x;
	robot_pose_y = msg.position.y;
	robot_pose_z = msg.orientation.z;
	robot_pose_w = msg.orientation.w;
}

//---CALLBACK TO READ THE QRCODE AND INTERPRETATE THE MOVEMENT ASKED---//
void barcodeCallback(const std_msgs::String& msg){

	//THE QRCODE CALLBACK ONLY RUNS IF THE FLAG BARCODE IS FALSE. THIS ENABLES TO RESTRICT THE USE OF QRCODE READER. IN THIS WAY THE ROBOT CANT, BY MISTAKE, CATCH A QRCODE BECAUSE IN EACH MOVEMENT WE PUT THIS FLAG TO TRUE
	if(barcode)
		return;

	std::string power_s;
	std::string energy_s;
	std::string label_s;
	std::string point;
	std::string str;
	int room;

	count_lines = 0;

	double power;
	double energy;
	
	Point pt;

	char* pch;
	char buff[120];
	memcpy(buff, msg.data.c_str(), msg.data.size()); //STORE THE DATA FROM QRCODE TO BUFF

	//WE IMPLEMENTED A WAY TO SECTIONATE THE STRING TO TAKE PARTS EQUIVELENTS TO THE ELEMENTS READ

	// POWER VALUE
	pch = strtok(buff, "W");
	str = std::string(pch);
	
	unsigned first = str.find(" ");
	unsigned last = str.find("W");

	power_s = str.substr(first,last-first);
	power = stod(power_s); //STORES THE PART OF THE VAULE ON THE RESPECTIVE LINE
	
	// ENERGY VALUE
	pch = strtok(NULL, "J");
	str = std::string(pch);

	first = str.find(" ");
	last = str.find("J");

	energy_s = str.substr(first,last-first);
	energy = stod(energy_s); //STORES THE PART OF THE VAULE ON THE RESPECTIVE LINE

	ROS_INFO("Power: %2.2f W and Energy: %2.2f J required for room.\n", power, energy);
	

	pch = strtok(NULL, "\n");
	str = std::string(pch);

	//SELECT THE MODES READ ON THE QRCODE

	if(extractIntegerWords(str))
	{
		//THIS STORES THE FIRST ROOM TO DESINFECT, THAT IS ON THE QRCODE
		//AND ENABLES THE FLAGS TO DO THIS MOVEMENT
		//THIS FLAGS WILL BE USED ON THE MAIN WHILE, TO DEFINE WHAT MOVEMENT TO DO IN LOOP
		ROS_INFO("FW!");

		pch = strtok(NULL, ":");
		pch = strtok(NULL, ",");

		room = atof(pch);

		if(room == 1)
		{
			fw_flag = 1; //MOVEMENT FLAG
			room_flag = 1; //ROOM FLAG
		}
		else if(room == 2)
		{
			fw_flag = 1;
			room_flag = 2;
		}
	}
	else{
		if (str == "FWY")
		{
			//THIS STORES THE POINTS ON THE QRCODE TO A ARRAY, THIS FACILITATES THE USE OF THEM ON OTHER FUNCTIONS
			//ENABLES THE FLAGS TO DO THIS MOVEMENT
			//THIS FLAGS WILL BE USED ON THE MAIN WHILE, TO DEFINE WHAT MOVEMENT TO DO IN LOOP
			ROS_INFO("FWY!");
			fwy_flag = 1; //MOVEMENT FLAG
			for (int l = 0; l < sizeof(buff); l++)
			{
				if(buff[l] == '\n')
					count_lines++;
			}
			for (int a = 0; a < count_lines-1; a++)
			{
				if(a == 0)
				{
					pch = strtok(NULL, ",");
				}
				else
				{
					pch = strtok(NULL, "(");
					pch = strtok(NULL, ",");
				}

				pt.x = atof(pch);

				pch = strtok(NULL, ")");
				
				pt.y = atof(pch);

				ROS_INFO("ptx e pty: %2.2f %2.2f", pt.x, pt.y);
				points_database[a].x = pt.x;
				points_database[a].y = pt.y;
				points_database[a].yaw = 0;
				points_database[a].label = a;
			}
		}else
		{
			//THIS STORES THE POINTS ON THE QRCODE TO A ARRAY, THIS FACILITATES THE USE OF THEM ON OTHER FUNCTIONS
			//ENABLES THE FLAGS TO DO THIS MOVEMENT
			//THIS FLAGS WILL BE USED ON THE MAIN WHILE, TO DEFINE WHAT MOVEMENT TO DO IN LOOP
			pch = strtok(NULL, "\n");
			str = std::string(pch);

			ROS_INFO("DR!");
			dr_flag = 1; //MOVEMENT FLAG
			pch = strtok(NULL, ":");
			pch = strtok(NULL, ",");
			room = atof(pch); //ROOM VARIABLE

			if(room == 1)
			{
				room_flag = 1; //ROOM FLAG
				//ARRAY TO STORE THE ROOM CENTER POINTS
				points_database[0].x = 1.5;
				points_database[0].y = -2;
				points_database[0].yaw = 0;
				points_database[0].label = 1;
				points_database[1].x = 4;
				points_database[1].y = -3;
				points_database[1].yaw = 0;
				points_database[1].label = 2;
			}
			else
			{
				room_flag = 2;
				points_database[0].x = 4;
				points_database[0].y = -3;
				points_database[0].yaw = 0;
				points_database[0].label = 2;
				points_database[1].x = 1.5;
				points_database[1].y = -2;
				points_database[1].yaw = 0;
				points_database[1].label = 1;
			}
			ROS_DEBUG("Moving to Center of the Room: %d", room);
		}

	}

	barcode = 1;
}

//---CALLBACK TO RECEIVE VALUES FROM THE LASER---//
//THIS CALLBACK WILL RECEIVE THE LASER SCANS, ELIMINATE THE "INF" AND "NAN" VALUES TO THE RESPECTIVE LIMITS AND SEPARATE IN SECTIONS THE READS
//THIS WILL BE USEFULL TO THE FOLLOW WALL IMPLEMENTATION
void callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::vector<float> laser_ranges = msg->ranges;

  for(int i = 0; i < 513; i++){
    if(isnan(laser_ranges[i])){
      laser_ranges[i] = 5.59;
    }
    if(laser_ranges[i] < 0.01){
      laser_ranges[i] = 5.59;
    }
  }

  section.right = *std::min_element(laser_ranges.begin() , laser_ranges.begin() + 147);
  section.front = *std::min_element(laser_ranges.begin() + 157, laser_ranges.begin() + 357);
  section.left = *std::min_element(laser_ranges.begin() + 367, laser_ranges.begin() + 513);

  //ROS_INFO("front %f", section.front);
  //ROS_INFO("left %f", section.left);
  //ROS_INFO("right %f", section.right);
}

//---CALLBACK TO RECEIVE FROM DETECTION PACKAGE IF IT DETECTED A PERSON---//
void detector(const pal_detection_msgs::Detection2d& msg)
{
	if(msg.x == 0)
	{
		detected = 0;
	}else
	{
		detected = 1;
	}

}

//--------------------------------------MOVEMENT FUNCTIONS----------------------------------------------//

//---FUNCTION OF MOVE_BASE TO MAKE ROBOT MOVE---//
void moveTo(double x, double y){

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  ac("move_base", true);
    //WAIT FOR THE SERVER OF MOVE_BASE TO COME UP
    while( !ac.waitForServer(ros::Duration(5.0)) ){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
  
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Next Waypoint %2.2f %2.2f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

	//IF A PERSON IS DETECTED WHEN MOVE BASE IS CALLED IT WILL STOP THE DESINFECTION AND WAIT
	//THIS IS DETERMINED BY "DESINFECAO" FLAG. IF THIS FLAG IS FALSE, THE ROBOT WILL NOT MOVE ON FWY AND DR MOVEMENTS
	if(detected == 1)
	{
		desinfecao = 0;
		return;
	}
    
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		if(x == 0 && y == 0) // IF THE ROBOT IS IN THE INITIAL POSE TI WILL NOT START DESINFECTING
			desinfecao = 0;
		else
			desinfecao = 1;
		a++; //VARIABLE "a" IS A COUNTER OF POINTS DESINFECTED. THIS IS NEEDED TO FWY TO KNOW WHEN THE ROBOT STOPS THE DESINFECTION AND SEND HIM TO INICIAL POINT
	}
	else
	{
		//IF THE ROBOT DOESNT REACH THE GOAL, IT WILL INCREMENT THE "a" VARIABLE AND SEND A ERROR. THE INCREMENTATION IS TO THE ALGORITHM PASS TO THE FOLLOWING POINT, AND FORGET THE POIN TNO REACHABLE
		a++;
		desinfecao = 0;
		ROS_INFO("Error reaching destination!");
	}
    return;
}

//---FOLLOW THE WAY POINTS---//
//THIS MOVEMENT IS DEVIDED INTO TWO FUNCTION:
//"fwy" IS TO SEND THE GOALS TO THE ROBOT, THIS GOALS ARE STORED ON "points_database" ARRAY AND THE POINT IS DETERMINED BY THE COUNTER "a"
void fwy()
{
	//IF THE ROBOT IS ON THE LAST POINT (THE IF CONDITION) IT WILL PUT THE FLAG MOVEMENTS AT FALSE AND ENABLES THE QRCODE TO BE USED AFTERWARDS.
	if(a == count_lines-1)
	{
		a = 0;
		fwy_flag = 0;
		barcode = 0;
		moveTo(0, 0);
		a = 0;
		//THIS LINE IS TO STOP THE ROS NODE. THE ONLY PURPOSE IS FOR THE APRESENTATION, IT CAN BE REMOVED AND THE ROBOT WILL READ THE QRCODE TO START ANOTHER MOVEMENT
		ros::shutdown();
		//return;
	}
	else
		moveTo(points_database[a].x, points_database[a].y);

}

//THIS FUNCTION IS TO RESTRING THE DESINFECTION DEPENDING ON PERSON DETECTION. IF A PERSON IS DETECTED IT WILL SPAM A ERROR TO THE CONSOLE, IF NOT IT WILL START DESINFECTION
//TO PREVENT BUGS, EVERY ITERATION IT PUTS THE FLAGS TO OTHER MOVEMENTS AT FALSE.
//THE ONLY FUNCTION THAT MAKES THE FLAG MOVEMNTS AT TRUE IS THE QRCODE FALLBACK. TO ACTIVATE SPECIFIC FUCNTIONS.
void fwy_detected()
{
	if(detected == 1)
	{
		ROS_INFO("Please get out of the way!");
	}
	else
	{
		if(desinfecao == 1)
		{
			ROS_INFO("UV lamp ON");
			ros::Duration(2.0).sleep(); //DESINFECTION SIMULATION. STOPS THE ROBOT FOR 2 SECONDS.
			ROS_INFO("UV lamp OFF");
		}
	}
	dr_flag = 0;
	fw_flag = 0;
}

//---DESINFECT ROOM ON THE CENTER POINT---//
//THIS MOVEMNT IS THE MORE BASIC. IT SEND A POSITION TO THE ROBOT, AND DESINFECT THE ROOM WHEN IT GETS THERE.
void dr()
{
	ROS_INFO("Begin Room: %d", room_flag);
	moveTo(points_database[0].x, points_database[0].y); //FIRST POINT OF THE MOVEMNT STORED ON THE ARRAY. IT IS STORED BY QRCODE CALLBACK
	ROS_INFO("Iluminating the room!");
	ros::Duration(2.0).sleep();
	ROS_INFO("Continue to other room");
	moveTo(points_database[1].x, points_database[1].y); //SECOND POINT
	ROS_INFO("Iluminating the room!");
	ros::Duration(2.0).sleep();
	ROS_INFO("Return to Intial Position");
	moveTo(0, 0);
	fwy_flag = 0;
	fw_flag = 0;
	dr_flag = 0;
	barcode = 0; //ENALBES THE QRCODE TO BE USED AGAIN
	a = 0;
	ros::shutdown();
}

//---FOLLOW WALL---//

//MAP STATES TO FOLLOW WALL ACTION INFORMATION
std::map<int, std::string> state_dict_ = {
  {0, "Find wall"},
  {1, "Turn right"},
  {2, "Follow the wall"},
  {3, "Turn left"},
  {4, "Diagonally right"},
  {5, "Diagonally left"},
  {6, "STOP"}
};

//THIS CHANGES THE STATE OF THE FOLLOW WALL MOVEMNTS AND SENDS THE INFORMATION TO CONSOLE
void change_state(int state)
{
  if (state != state_)
  {
    ROS_INFO("State of Bot - [%d] - %s", state, state_dict_[state].c_str());
    state_ = state;
  }
}

//THIS FUNCTION IS TO DECIDE WITH ROOM TO BEGIN THE FOLLOW WALL AND TO SWAP THE ROOM AFTER IT FINISHES.
void fw()
{
	if(room_flag == 1 && fw_running == 0) //IT ONLY SWAPS OR INICIATE IF FOLLOW WALL FLAG IS FALSE
	{
		ROS_INFO("Begin Room: %d", room_flag);
		moveTo(-0.5,-1.5); // THIS IS A POINT INSIDE THE ROOM, WHERE THE FOLLOW WALL STARTS
		fw_running = 1; //START FOLLOW WALL
	}
	else if(room_flag == 2 && fw_running == 0)
	{
		ROS_INFO("Begin Room: %d", room_flag);
		moveTo(3,-1.5);
		fw_running = 1;
	}
}

//
void fw_movement()
{
	if(fw_running == 1)
	{
		float b = 1.0; // maximum threshold distance
		float a = 0.5; // minimum threshold distance
		//geometry_msgs::Twist velocity;
		float linear_x = 0;
		float angular_z = 0;

		ROS_INFO("follow_direction %d", follow_dir);
		//IF ALL REGIONS ONF THE LASER ARE BIGGER THEN THE max threshold IT WILL CHANGE STATE TO FIND THE WALL
		if (section.front > b && section.left > b && section.right > b)
		{
			// Reset follow_dir
			change_state(0);
			ROS_INFO("Reset Follow_dir");
		}
		else if (follow_dir == -1)  // To set the direction of wall to follow
		{
			if (section.left < b)
			{
			change_state(1);
			follow_dir = 0;
			ROS_INFO("following left wall");
			}
			else if (section.right < b)
			{
			change_state(3);
			follow_dir = 1;
			ROS_INFO("following right wall");
			}
			else
			{
			change_state(2);
			ROS_INFO("following front wall");
			}
		}
		else if(section.front < a && section.left < a && section.right < a){ //IF ALL REGIONS ARE TOO CLOSE TO THE WALL THE ROBOT STOPS
			ROS_INFO("Too Close");
			change_state(6);
			ROS_INFO("STOP");
		}else{
			ROS_INFO("Running");
		}
		
		if (follow_dir == 0) // Algorithm for left wall follower
		{
			//ACTIONS TO DO DEPENDING ON THE LASER RANGE
			if (section.left > b && section.front > a)
			{
				change_state(5);
			}
			else if (section.left < b && section.front > a)
			{
				change_state(2);
			}
			else if(section.left < b && section.front < a)
			{
				change_state(1);
			}
			else if(section.left < a)
			{
				change_state(4);
			}
			else
			{
				ROS_INFO("Follow left wall is not running");
			}
		}
		else if (follow_dir == 1) // Algorithm for right wall follower
		{
			if (section.right > b && section.front > a)
			{
				change_state(4);
			}
			else if (section.right < b && section.front > a)
			{
				change_state(2);
			}else if (section.right < b && section.front < a){
				change_state(3);
			}else if(section.right < a){
				change_state(5);
			}else{
				ROS_INFO("Follow right wall is not running");
			}
		}

		//VELOCITIES TO EACH ACTION
		if (state_ == 0)
		{
			linear_x = 0.25;
			angular_z = 0.0;
		}
		else if (state_ == 1)
		{
			linear_x = 0.0;
			angular_z = -0.3;
		}
		else if (state_ == 2)
		{
			linear_x = 0.25;
			angular_z = 0.0;
		}
		else if (state_ == 3)
		{
			linear_x = 0.0;
			angular_z = 0.3;
		}
		else if (state_ == 4)
		{
			linear_x = 0.1;
			angular_z = -0.5;
		}
		else if (state_ == 5)
		{
			linear_x = 0.1;
			angular_z = 0.5;
		}else if(state_ == 6){
			linear_x = 0.0;
			angular_z = 0.0;
		}else{
			ROS_INFO("Unknown state!");
		}

		velocity.linear.x = linear_x;
		velocity.angular.z = angular_z;
		motor_command_publisher.publish(velocity); //PUBLISH THE VELOCITIES TO "cmd_vel" TOPIC

		if((robot_pose_x < 1.0) &&  (robot_pose_x > -0.5) &&  (robot_pose_y > -0.5)  && (robot_pose_y < 1.0)) //DETECT IF THE ROBOT FINISHED DESINFECTED ONE ROOM
		{
			std::cout << fw_room << std::endl;
			room_flag = 2; //SWAP THE ROOM FLAG TO CHANGE ROOM
			fw_running = 0; //DISABLES THE MOVEMENT UNTIL IT REACHES THE OTHER ROOM
			fw_room++; //ROOM COUNTER
			if(fw_room == 2) //IF THE ROOM COUNTER EQUALS 2 MEANS THAT THE ROBOT DESINFECTED THE TWO ROOM AND STOPS THE MOVEMENT
			{
				moveTo(0, 0); //SEND THE ROBOT TO INITIAL POSITION
				//PUT THE MOVEMENT FLAGS TO FALSE
				fw_running = 0;
				fw_room = 0;
				fwy_flag = 0;
				fw_flag = 0;
				dr_flag = 0;
				barcode = 0; //ENABLES QRCODE
				a = 0; //RESET MOVE BASE COUNTER
				ros::shutdown();
			}
		}
		else if((robot_pose_x < 3.0) && ( robot_pose_x > 2.5) && (robot_pose_y > -0.5)  && (robot_pose_y < 1.0))
		{	
			room_flag = 1;
			fw_running = 0;
			fw_room++;
			if(fw_room == 2)
			{	
				moveTo(0, 0);
				fw_running = 0;
				fw_room = 0;
				fwy_flag = 0;
				fw_flag = 0;
				dr_flag = 0;
				barcode = 0;
				a = 0;
				ros::shutdown(); //REMOVE TO NEVER STOP THE SYSTEM BETWEEN QRCODES READS
			}
		}
	}
}


int main(int argc, char **argv){
	ros::init(argc, argv, "guide_node");
	ros::NodeHandle nh;
	
	//SUBSCRIBERS AND PUBLISHERS ZONE
	ros::Subscriber subs_barcode = nh.subscribe("/barcode",1,barcodeCallback);

	ros::Subscriber person_detect = nh.subscribe("/Detected", 1, detector);

	motor_command_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	ros::Subscriber laser_sub = nh.subscribe("scan", 10, callback_laser);

	ros::Subscriber robot_pose = nh.subscribe("robot_pose", 1, callback_pose);
	
	ros::Rate rate(10);

	//MAIN LOOP TO CHANGE THE ACTIONS.
	//IN SUMMARY, THE LOOP WILL CONSTANTLY BE PALYING WITH FLAGS THAT WILL DETERMINATE WICTH FUNCTION THE CODE WILL RUN. EACH MOVEMENT AS THE OWN FLAG, THAT WILL BE ENABLED ON QRCODE READS.
	//THIS LOOP MANTAINS THE SAME ALGORITHM TO THE ALL MOVEMENT, AND IS ENOUGH CHANGING THE FLAG TO CHANGE THE ALGORITHM.
	while(ros::ok)
	{
		//IF THE BARCODE FLAG IS FALSE. IT MEANS THAT THE QRCODE WAS NOT READ. SO IT WILL START THE MOVEMEN TO FIND IT ON INICIAL POSE.
		if(!barcode)
		{
			ROS_INFO("Searching QrCode!");
			velocity.linear.x = 0.0;
        	velocity.angular.z = 0.2;
			motor_command_publisher.publish(velocity);
    		usleep(1500);
		}
		else //AFTER IT FINDS THE QRCODE, IT WILL DO THE CALLBACK THAT WILL DETERMINATE WITCH MOVEMENT IS ASKED. THIS IS KNOWND BY THE MOVEMENT FLAGS
		{
			if(fwy_flag == 1)
			{
				//THIST SPAM ON "ros::spinOnce()" IS BECAUSE THE INTERNET CONNECTION IS CONSTANTLY FAILLING. AND TO REDUCE THE ERROR WE SPAMMED THIS LINE TO FORCE ROS TO SPIN THROUGH THE CODE MORE TIMES
				//THIS IS USEFUL TO GIVE TIME TO THE CAMERA TO CATCH UP WITH THE MOVEMNT. WITH LOW CONNECTIVITY THE CAMERA AS TOO MUCH LAG
				//THIS HELPS TO THE CAMERA TO FIND PEOPLE AT TIME BEFORE RUNNING THE DESINFECTION FUNCTION
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				fwy();
				//if(a != count_lines)
				//{
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				ros::spinOnce();
				rate.sleep();
				fwy_detected();
				//}
			}
			else if(dr_flag == 1)
			{
				dr();
			}
			else if(fw_flag == 1)
			{
				fw();
				ros::spinOnce();
				rate.sleep();
				fw_movement();
			}

		}
		ros::spinOnce();
		rate.sleep();
	}

}
