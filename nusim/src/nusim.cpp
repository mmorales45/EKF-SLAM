#include"ros/ros.h"
#include"std_msgs/UInt64.h"
#include <iostream>
#include <ros/console.h>

class Sim
{
    public:
        Sim() {
            data_val = 0;
            timestep.data = 0;
            nh.getParam("/nusim/rate",rate);
            pub = nh.advertise<std_msgs::UInt64>("UInt64", 100);
            ros::ServiceServer nh.advertiseService(const std::string& service, reset);

            
            main_loop();
            // while(ros::ok()) {
            //     timestep.data++;
            //     timer = nh.createTimer(ros::Duration(1/rate), &Sim::main_loop, this);
            // }

    
        }
           
        //  void main_loop(const ros::TimerEvent &) const
        //  {
        //     // implement the state machine here
        //     // data_val = data_val +1;
        //     // std::cout << data_val << "\n";
        //     pub.publish(timestep);
        //     //  ROS_INFO("Hello ");
        //  }
        void main_loop() {
            ros::Rate new_rate(rate);
            while(ros::ok()) {
                timestep.data++;
                pub.publish(timestep);
                new_rate.sleep();
            }
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Timer timer;
        double rate;
        std_msgs::UInt64 timestep;
        int data_val;


};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nodename");
    Sim node;
    ros::spin();
    return 0;
}