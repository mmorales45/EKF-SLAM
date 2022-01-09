#include"ros/ros.h"
#include"std_msgs/UInt64.h"
#include <iostream>
#include <ros/console.h>
#include <std_srvs/Empty.h>

class Sim
{
    public:
        Sim() {
            data_val = 0;
            timestep.data = 0;
            nh.getParam("/nusim/rate",rate);
            pub = nh.advertise<std_msgs::UInt64>("UInt64", 100);
            timer = nh.createTimer(ros::Duration(1/rate), &Sim::main_loop, this);            
        }
           
        bool reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
        {
        timestep.data = 0;
        pub.publish(timestep);
        return true;
        }

        void main_loop(const ros::TimerEvent &)
         {
            // implement the state machine here
            timestep.data++;
            pub.publish(timestep);  
         }


    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Timer timer;
        double rate;
        std_msgs::UInt64 timestep;
        int data_val;
        ros::ServiceServer service = nh.advertiseService("my_service", &Sim::reset, this);


};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nodename");
    Sim node;
    ros::spin();
    return 0;
}