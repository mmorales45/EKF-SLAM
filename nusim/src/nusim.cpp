#include"ros/ros.h"
#include"std_msgs/UInt64.h"
class Sim
{
    public:
        Sim():
        nh{},
        pub(nh.advertise<std_msgs::UInt64>("Stdmsg",5)),
        timer(nh.createTimer(ros::Duration(0.1), &Sim::main_loop,this))
        {
        }
    void main_loop(const ros::TimerEvent &) const 
    {
        pub.publish(std_msgs::UInt64);
    }

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Timer timer;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "TurtleSim");
    MyNode node;
    ros::spin();
    return 0;
}
