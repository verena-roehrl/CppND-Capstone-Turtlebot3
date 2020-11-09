#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <vector>
#include <stdlib.h>
#include <sstream>

class Turtlebot{
    private:
        ros::NodeHandle n;
        ros::ServiceClient spawnClient;
        std::string turtlebot_name{"turtlebot"};
        float x_initial{1.0};
        float y_initial{-1.0};
    public:
        Turtlebot() {}
        Turtlebot(std::string name, float x, float y)
            : turtlebot_name(name), x_initial(x), y_initial(y)
        {
            this->spawnClient = n.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model", false);
            //spawn urdf
            gazebo_msgs::SpawnModel srv;
            srv.request.initial_pose.position.x = x_initial;
            srv.request.initial_pose.position.y = y_initial;
            srv.request.initial_pose.position.z = 0.0;     
            srv.request.initial_pose.orientation.x = 0.0;
            srv.request.initial_pose.orientation.y = 0.0;
            srv.request.initial_pose.orientation.z = 0.0;
            srv.request.initial_pose.orientation.w = 1.0;     

            srv.request.model_name = turtlebot_name;
            srv.request.reference_frame="world";
            srv.request.robot_namespace = turtlebot_name;

            std::string package_path = ros::package::getPath("turtlebot3_gazebo_custom");

            std::string urdf_path = package_path + std::string("/urdf/turtlebot3_waffle.urdf");
            std::ifstream file(urdf_path);
            std::string line;
  
            // Parse the contents of the given urdf in a string
            while(!file.eof()) 
            {
                std::getline(file,line);
                srv.request.model_xml+=line;
            }
            file.close();

            spawnClient.waitForExistence();
            if (spawnClient.call(srv))
            {
                ROS_INFO("Successfully spawned URDF model %s", srv.request.model_name.c_str());
                
                // Launch turtlebot_behaviour nodes in new terminals
                std::stringstream terminal_command_behaviour_ss;
                terminal_command_behaviour_ss << "x-terminal-emulator -e \"roslaunch turtlebot3_movement_manager turtlebot_behaviour.launch namespace:=\"" << turtlebot_name  << "\"\"";
                std::string terminal_command_behaviour = terminal_command_behaviour_ss.str();
                system(terminal_command_behaviour.c_str());
                
                std::stringstream terminal_command_dependencies_ss;
                terminal_command_dependencies_ss << "x-terminal-emulator -e \"roslaunch turtlebot3_movement_manager turtlebot_behaviour_dependencies.launch namespace:=\"" << turtlebot_name  << "\" x_init:=\"" << std::to_string(x_initial) << "\" y_init:=\"" << std::to_string(y_initial) << "\"\"";
                std::string terminal_command_dependencies = terminal_command_dependencies_ss.str();
                system(terminal_command_dependencies.c_str());
                
                
            }
            else
            {
                ROS_ERROR("Failed to call /gazebo/spawn_urdf_model service");
            }
        }
};

class Manager
{
    private:

    public:
        Manager()
        {
        }

        void spawn()
        {
            std::string model_name;
            std::string x_string;
            std::string y_string;
            std::string z_string;
            ROS_INFO("Insert a name for your Turtlebot");
            std::cin >> model_name;
            ROS_INFO("Insert initial x coordinate for your Turtlebot %s [only floating point values -5 < x < 5]", model_name.c_str());
            std::cin >> x_string;
            ROS_INFO("Insert initial y coordinate for your Turtlebot %s [only floating point values -7 < y < 7]", model_name.c_str());
            std::cin >> y_string;

            Turtlebot* turtlebot = new Turtlebot(model_name, stof(x_string), stof(y_string));
        }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "manager");
    ros::NodeHandle nh;
    Manager manager = Manager();
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