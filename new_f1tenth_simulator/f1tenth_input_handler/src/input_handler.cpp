#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <termios.h>
using namespace std;

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "keyboard_listener");
    ros::NodeHandle n;
    ros::Publisher input_pub;
    n = ros::NodeHandle("~");
    input_pub = n.advertise<std_msgs::Int64>("/simulator/command/input", 1);
    while (ros::ok())
    {
      int c = getch();
      char character = c;
      ROS_INFO(("Read Key: " + to_string(character)).c_str());
      std_msgs::Int64 valueToPublish;
      int64_t dataValue = (int64_t)c;
      valueToPublish.data = dataValue;
      input_pub.publish(valueToPublish);
    }
    return 0;
}
