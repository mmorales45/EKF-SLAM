#include"ros/ros.h"
#include"std_msgs/UInt64.h"

class Sim
{
    public:
        Sim():
            nh{},
            pub(nh.advertise<std_msgs::UInt64>("js", 5)),
            timer(nh.createTimer(ros::Duration(0.002), &Sim::main_loop, this))
            // nh().getParam("private_name", param)
         {
         }


         void main_loop(const ros::TimerEvent &) const
         {
            // implement the state machine here
             pub.publish(std_msgs::UInt64{});
         }

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Timer timer;
        // std::string param;

};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nodename");
    Sim node;
    ros::spin();
    return 0;
}