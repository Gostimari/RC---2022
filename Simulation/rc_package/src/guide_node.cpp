#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <map>
#include <fstream>
#include <string>

struct Point{
    double x; 
    double y;
	double yaw;
	int label;
};

Point qrcode_database[4];
Point points_database[5];
int qr_label;

int a = 0;
int i = 0; // Guarda o indice do qrcode atual
bool to_point = 0; //true se o robô chegou ao QRcode


void moveToQRCode(double x, double y, double t);
void moveTo(double x, double y);


void barcodeCallback(const std_msgs::String& msg){


	if(!to_point)
		return;

	std::string power_s;
	std::string energy_s;
	std::string label_s;
	std::string point;
	std::string str;

	double power;
	double energy;
	
	Point pt;

	char* pch;
	char buff[120];
	memcpy(buff, msg.data.c_str(), msg.data.size());

	pch = strtok(buff, " ");
	pch = strtok(NULL, "\n");

	qr_label = atoi(pch);

	ROS_INFO("Start Desinfection Room %d\n",qr_label);

	// LP value
	pch = strtok(NULL, "W");
	str = std::string(pch);
	
	unsigned first = str.find(" ");
	unsigned last = str.find("W");

	power_s = str.substr(first,last-first);
	power = stod(power_s);

	//ROS_INFO("Power string: %2.2f\n",power);
	
	// V value
	pch = strtok(NULL, "J");
	str = std::string(pch);

	first = str.find(" ");
	last = str.find("J");

	energy_s = str.substr(first,last-first);
	energy = stod(energy_s);

	//ROS_INFO("Energy string: %2.2f\n",energy);

	ROS_INFO("Power: %2.2f W and Energy: %2.2f J required for room.\n", power, energy);
	
	// Points one by one

	pch = strtok(NULL, "Y");
	
	for (int i = 0; i < 5; i++)
	{	
		pch = strtok(NULL, "(");
		pch = strtok(NULL, ",");

		pt.x = atof(pch);

		pch = strtok(NULL, ")");
		
		pt.y = atof(pch);
		ROS_DEBUG("ptx e pty: %2.2f %2.2f", pt.x, pt.y);
		points_database[i] = pt;
		moveTo(pt.x, pt.y);
	}

	to_point = 0;
	
}

void moveTo(double x, double y){
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  ac("move_base", true);
    //wait for the action server to come up
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
    
    ac.sendGoal(goal);

    ac.waitForResult();
    //bool result = ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //if (result)
    {
        ROS_INFO("UV lamp ON");
		ros::Duration(2.0).sleep();
		ROS_INFO("UV lamp OFF");
    }
    else
        ROS_INFO("Error reaching destination!");
    return;
}



void moveToQRCode(double x, double y, double t){
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  ac("move_base", true);
    //wait for the action server to come up
    while( !ac.waitForServer(ros::Duration(5.0)) ){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
  
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();


    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;

	if(2.0 < t < 3.0)
	{
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 1.0;
		goal.target_pose.pose.orientation.w = 0.0;
	}
	else if(0.35 < t < 0.45)
	{
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.7;
		goal.target_pose.pose.orientation.w = 0.7;
	}
	else if(t < 2.0)
	{
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.7;
		goal.target_pose.pose.orientation.w = -0.7;
	}
	else
	{	
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.0;
		goal.target_pose.pose.orientation.w = 1.0;
	}

    ROS_INFO("Next Waypoint %2.2f %2.2f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

	ac.sendGoalAndWait(goal);


    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)

    {
        to_point = 1;
		i += 1;
    }
    else{
		moveTo(x, y);
        ROS_INFO("Error reaching destination!");
	}
    return;
	
}


bool sort_file (Point a, Point b)
{
	if (a.label < b.label)
		return 1;
	else
		return 0;

}



int main(int argc, char **argv){
	ros::init(argc, argv, "guide_node");
	ros::NodeHandle nh;
	
	ros::Subscriber subs_barcode = nh.subscribe("/barcode",1,barcodeCallback);

	std::ifstream file_qrcodedb("./RC-project/qrcode_database.csv", std::ifstream::in);
	if (!file_qrcodedb.is_open()){
		ROS_ERROR("Could not open QRCode database!");
		return -1;
	}

	Point pt;
	std::string line;
	int w = 0;
	int z = 0;
	while (std::getline(file_qrcodedb, line)){

		char* pch;
		char buf[120];
		memcpy(buf, line.c_str(), line.size());

		pch = strtok(buf, ",");
		pt.label = atoi(pch);

		pch = strtok(NULL, ",");
		pt.x = atof(pch);
		pch = strtok(NULL, ",");
		pt.y = atof(pch);
		pch = strtok(NULL, "\n");
		pt.yaw = atof(pch);

		qrcode_database[w] = pt;

		w++;
	}
	
	file_qrcodedb.close();

	int len = sizeof(qrcode_database->label)/sizeof(qrcode_database[0].label);

	std::sort(qrcode_database, qrcode_database+w, sort_file);

	
	ros::Rate rate(10);



	while(ros::ok){
	
		if (!to_point)
		{
			if (i == 4)
			{
				ROS_INFO("Desinfection Over!\n");
				ROS_INFO("Shutting Down\n");
				ros::shutdown();
			}

			moveToQRCode(qrcode_database[i].x, qrcode_database[i].y, qrcode_database[i].yaw);
		}

		ros::spinOnce();
		rate.sleep();
		
	}


}
