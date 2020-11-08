#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class TurtlebotBehaviour{
    private:
        ros::NodeHandle n;
        MoveBaseClient moveBaseClient;
        
        std::string turtlebot_name;
        std::string move_base_topic;
    public:
        TurtlebotBehaviour(std::string goal_topic) : move_base_topic(goal_topic), moveBaseClient(move_base_topic, true)
        {
            //wait for the action server to come up
            /*
            while(!moveBaseClient.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO(move_base_topic.c_str());
                ROS_INFO("Waiting for the move_base action server to come up");            
            }      
            ROS_INFO("move_base action server is up");
            */
        }


        

        void autonomousNavigation()
        {
            std::string x_string;
            std::string y_string;
            ROS_INFO("Insert desired x coordinate for your Turtlebot %s as goal", turtlebot_name.c_str());
            std::cin >> x_string;
            ROS_INFO("Insert desired x coordinate for your Turtlebot %s as goal", turtlebot_name.c_str());
            std::cin >> y_string;

            while(!moveBaseClient.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }

            move_base_msgs::MoveBaseGoal goal;

            //we'll send a goal to the robot to move 1 meter forward
            goal.target_pose.header.frame_id = "leo/base_link";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = stof(x_string);
            goal.target_pose.pose.position.y = stof(y_string);
            goal.target_pose.pose.orientation.w = 1.0;

            ROS_INFO("Sending goal");
            moveBaseClient.sendGoal(goal);

            moveBaseClient.waitForResult();

            if(moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Hooray, the base moved 1 meter forward");
            }
            else
            {
            ROS_INFO("The base failed");
            }
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlebot_behaviour");
    ros::NodeHandle nh;
    std::string turtlebot_name;
    std::string goal;
    nh.getParam(ros::this_node::getName() + "/namespace", turtlebot_name);
    goal = turtlebot_name + "/move_base";
    TurtlebotBehaviour* turtlebotBehaviour = new TurtlebotBehaviour(goal);
    while(ros::ok())
    {
        ROS_INFO("Press a for autonomous navigation of Turtlebot3");
        if(getchar() == 'a')
        {
            turtlebotBehaviour->autonomousNavigation();
        }
    }
    
    ros::spin();
    return 0;
}