#include "ros/ros.h"
#include "std_msgs/Char.h"

#include <unistd.h>
#include <termios.h>

#include <sstream>


// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

std_msgs::Char key;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_publisher");
  ros::NodeHandle n;

  ros::Publisher keyboard_publisher = n.advertise<std_msgs::Char>("keyboard_publisher", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    key.data = getch();

    if (key.data == '\x03')
    { 
        break;
    }
    keyboard_publisher.publish(key);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}