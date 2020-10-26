#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <iostream>
#include <fstream>
#include <ros/package.h>

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
            ROS_INFO("in spawn");
            gazebo_msgs::SpawnModel srv;
            
            srv.request.model_name ="test_model";
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
            ROS_INFO("Service ready");
            if (spawnClient.call(srv))
            {
                ROS_INFO("Successfully spawned URDF model %s", srv.request.model_name.c_str());
            }
            else
            {
                ROS_ERROR("Failed to call /gazebo/spawn_urdf_model service");
            }
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spawn_client");
    ros::NodeHandle nh;
    Manager manager = Manager(&nh);
    if(getchar())
    {
        ROS_INFO("in getchar");
        manager.spawn();     
    }
  
  ros::spin();
  
  return 0;
}