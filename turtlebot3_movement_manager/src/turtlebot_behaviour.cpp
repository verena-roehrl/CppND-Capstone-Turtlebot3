#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#define WAFFLE_MAX_LIN_VEL 0.26
#define WAFFLE_MAX_ANG_VEL 1.82
#define LIN_VEL_STEP_SIZE 0.01
#define ANG_VEL_STEP_SIZE 0.1

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class TurtlebotBehaviour{
    private:
        ros::NodeHandle nh;
        MoveBaseClient moveBaseClient;
        ros::Publisher cmdVelPub;
    
        std::string turtlebot_name;
        std::string move_base_topic;
        std::string cmd_vel_topic;

        int key;
        int status;
        float target_linear_vel;
        float target_angular_vel;
        float control_linear_vel;
        float control_angular_vel;
        geometry_msgs::Twist twist;

        const char* msg = R"(
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
space key, s : force stop
z : to go back to menu
"""
        )";

    public:
        TurtlebotBehaviour(std::string goal_topic, std::string cmd_topic, std::string name) 
        : move_base_topic(goal_topic), cmd_vel_topic(cmd_topic), turtlebot_name(name), moveBaseClient(goal_topic, true), status(0), target_linear_vel(0), target_angular_vel(0), control_linear_vel(0), control_angular_vel(0)
        {
            cmdVelPub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10); 
            //wait for the action server to come up
            while(!moveBaseClient.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the %s action server to come up", move_base_topic.c_str());            
            }      
            ROS_INFO("%s move_base action server is up", move_base_topic.c_str());
            
        }

        void autonomousNavigation()
        {
            std::string x_string;
            std::string y_string;
            ROS_INFO("Insert desired x coordinate for your Turtlebot %s as goal", turtlebot_name.c_str());
            std::cin >> x_string;
            ROS_INFO("Insert desired y coordinate for your Turtlebot %s as goal", turtlebot_name.c_str());
            std::cin >> y_string;

            while(!moveBaseClient.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the %s action server to come up", move_base_topic.c_str());
            }

            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = stof(x_string);
            goal.target_pose.pose.position.y = stof(y_string);
            goal.target_pose.pose.orientation.w = 1.0;

            moveBaseClient.sendGoal(goal);
            moveBaseClient.waitForResult();

            if(moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Turtlebot %s reached the desired goal", turtlebot_name.c_str());
            }
            else
            {
            ROS_INFO("Turtlebot %s failed to reach the desired goal", turtlebot_name.c_str());
            }
        }

        void teleoperation()
        {
            printf("%s", msg);
            while(true)
            {
                key = getKey();

                if(key == 'w')
                {
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE);
                    status = status + 1;
                    print_vel(target_linear_vel, target_angular_vel);
                }
                else if(key == 'x')
                {
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE);
                    status = status + 1;
                    print_vel(target_linear_vel, target_angular_vel);
                }
                else if(key == 'a')
                {
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE);
                    status = status + 1;
                    print_vel(target_linear_vel, target_angular_vel);
                }
                else if(key == 'd')
                {
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE);
                    status = status + 1;
                    print_vel(target_linear_vel, target_angular_vel);
                }
                else if(key == 's')
                {
                    target_linear_vel = 0.0;
                    control_linear_vel = 0.0;
                    target_angular_vel = 0.0;
                    control_angular_vel = 0.0;
                    print_vel(target_linear_vel, target_angular_vel);
                }
                else if(key == 'z')
                {
                    target_linear_vel = 0.0;
                    control_linear_vel = 0.0;
                    target_angular_vel = 0.0;
                    control_angular_vel = 0.0;
                    publish_twist(false);
                    break;
                }
            
                if(status == 20)
                {
                    printf("%s", msg);
                }

                publish_twist(true);
            }
        }

        int getKey()
        {
            int current_char;
            struct termios oldt;
            struct termios newt;

            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;

            newt.c_lflag &= ~(ICANON | ECHO);
            newt.c_iflag |= IGNBRK;
            newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
            newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
            newt.c_cc[VMIN] = 1;
            newt.c_cc[VTIME] = 0;
            tcsetattr(fileno(stdin), TCSANOW, &newt);

            current_char = getchar();

            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

            return current_char;
        }

        float checkLinearLimitVelocity(float vel)
        {
            float ret_vel = constrain(vel, -1 * WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL);
            return ret_vel;
        }

        float checkAngularLimitVelocity(float vel)
        {
            float ret_vel = constrain(vel, -1 * WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL);
            return ret_vel;
        }

        float constrain(float input, float low, float high)
        {
            float output = 0.0;
            if(input < low)
            {
                output = low;
            }
            else if(input > high)
            {
                output = high;
            }
            else
            {
                output = input;
            }
            return output;
        }

        float makeSimpleProfile(float control_vel, float target_vel, float step)
        {
            float output_vel;

            if(target_vel > control_vel)
            {
                output_vel = std::min(target_vel, control_vel + step);
            }
            else if(target_vel < control_vel)
            {
                output_vel = std::max(target_vel, control_vel - step);
            }
            else
            {
                output_vel = target_vel;
            }

            return output_vel;
        }

        void print_vel(float targ_linear_vel, float targ_angular_vel)
        {
            printf("\rCurrent: speed %f\tturn %f \r", targ_linear_vel, targ_angular_vel);
        }

        void publish_twist(bool running_mode)
        {
            if(running_mode)
            {
                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0));
                twist.linear.x = control_linear_vel;
                twist.linear.y = 0.0;
                twist.linear.z = 0.0;
                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0));
                twist.angular.x = 0.0;
                twist.angular.y = 0.0;
                twist.angular.z = control_angular_vel;
            }
            else
            {
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.linear.z = 0.0;
                twist.angular.x = 0.0;
                twist.angular.y = 0.0;
                twist.angular.z = 0.0;
            }
            cmdVelPub.publish(twist);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlebot_behaviour");
    std::string turtlebot_name = argv[1];
    std::string goal;
    std::string cmd_vel;
    char input;

    goal = "/" + turtlebot_name + "/move_base";
    cmd_vel = "/" + turtlebot_name + "/cmd_vel";

    TurtlebotBehaviour* turtlebotBehaviour = new TurtlebotBehaviour(goal, cmd_vel, turtlebot_name);
    while(ros::ok())
    {
        ROS_INFO("Press a for autonomous navigation  or t for teleoperation of your Turtlebot3 %s", turtlebot_name.c_str());
        
        input = getchar();
        if(input == 'a')
        {
            turtlebotBehaviour->autonomousNavigation();
        }
        else if(input == 't')
        {
            turtlebotBehaviour->teleoperation();
        }
    }
    
    ros::spin();
    return 0;
}