#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <vector>
#include <stdlib.h>

class Manager
{
    private:
        ros::ServiceClient spawnClient;

    public:
        Manager(ros::NodeHandle *nh)
        {
            //TODO check for initialization with initialization list
            this->spawnClient = nh->serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model", false);
        }

        void spawn()
        {
            std::string model_name;
            std::string x_string;
            std::string y_string;
            std::string z_string;
            ROS_INFO("Insert a name for your Turtlebot");
            std::cin >> model_name;
            ROS_INFO("Insert initial x coordinate for your Turtlebot");
            std::cin >> x_string;
            ROS_INFO("Insert initial y coordinate for your Turtlebot");
            std::cin >> y_string;
            ROS_INFO("Insert initial z coordinate for your Turtlebot");
            std::cin >> z_string;
            //geometry_msgs::Pose model_position = handlePositionUserInput(position_string);


            gazebo_msgs::SpawnModel srv;
            srv.request.initial_pose.position.x = stof(x_string);
            srv.request.initial_pose.position.y = stof(y_string);
            srv.request.initial_pose.position.z = stof(z_string);     
            srv.request.initial_pose.orientation.x = 0;
            srv.request.initial_pose.orientation.y = 0;
            srv.request.initial_pose.orientation.z = 0;
            srv.request.initial_pose.orientation.w = 1;     
            //ROS_INFO(model_name);

            srv.request.model_name = model_name;
            srv.request.reference_frame="world";

            std::string package_path = ros::package::getPath("turtlebot3_movement_manager");
            std::string urdf_path = package_path + std::string("/urdf/turtlebot3_burger.urdf");
            std::ifstream file(urdf_path);
            std::string line;
  
            while(!file.eof()) // Parse the contents of the given urdf in a string
            {
                std::getline(file,line);
                srv.request.model_xml+=line;
            }
            file.close();

            spawnClient.waitForExistence();
            if (spawnClient.call(srv))
            {
                ROS_INFO("Successfully spawned URDF model %s", srv.request.model_name.c_str());
            }
            else
            {
                ROS_ERROR("Failed to call /gazebo/spawn_urdf_model service");
            }
        }


        //TODO check why this does not work
        geometry_msgs::Pose handlePositionUserInput(std::string str)
        {
            geometry_msgs::Pose pose;
            std::vector<double> coordinates;
            std::istringstream ss(str); 

            // Traverse through all words 
            do { 
            // Read a word ERROR
                std::string coordinate; 
                ss >> coordinate; 
                ROS_INFO("%s", coordinate.c_str());
                coordinates.push_back(std::stof(coordinate));
                //pose.position.x = std::stod(coordinate) 
  
                // While there is more to read 
            } while (ss);


            pose.position.x = coordinates[0];
            pose.position.y = coordinates[1];
            pose.position.y = coordinates[2];
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 1;

            return pose;
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spawn_client");
    ros::NodeHandle nh;
    Manager manager = Manager(&nh);
    while(ros::ok())
    {
        ROS_INFO("Press s to spawn a new Turtlebot3");
        if(getchar() == 's')
        {
            manager.spawn();     
        }
    }
    
  
  ros::spin();
  
  return 0;
}