#include "ros/ros.h"
#include "std_msgs/String.h"
#include "systemcore/RedisMessage.h"
#include <ros/console.h>

#include <iostream>
#include <sstream>

#include "Rose_Eye.h"

Rose_Eye leftDisplay(64, 128);
Rose_Eye rightDisplay(64, 128);

void test1() {

    leftDisplay.eye->clearPoints();
    leftDisplay.iris->clearPoints();

    leftDisplay.eye->addPoint(20, 80, 0);
    leftDisplay.eye->addPoint(40, 80, 0);
    leftDisplay.eye->addPoint(40, 110, 0);
    leftDisplay.eye->addPoint(10, 120, 0);

    leftDisplay.iris->addPoint(30, 90, 0);
    leftDisplay.iris->addPoint(35, 100, 0);
    leftDisplay.iris->addPoint(30, 110, 0);

    leftDisplay.iris->setAnimationDuration(5000);
    leftDisplay.eye->setAnimationDuration(5000);

}

void testEye() {

    // leftDisplay.eye->clearPoints();
    // leftDisplay.iris->clearPoints();

    // leftDisplay.eye->addPoint(0, 0, 0);
    // leftDisplay.eye->addPoint(0, 0, 0);
    // leftDisplay.eye->addPoint(0, 0, 0);
    // leftDisplay.eye->addPoint(0, 0, 0);

    // leftDisplay.iris->addPoint(0, 0, 0);
    // leftDisplay.iris->addPoint(0, 0, 0);
    // leftDisplay.iris->addPoint(0, 0, 0);

    leftDisplay.eye->clearPoints();
    leftDisplay.iris->clearPoints();
    
    leftDisplay.eye->addPoint(10, 10, 0);
    //leftDisplay.eye->addPoint(50, 10, -1);
    leftDisplay.eye->addPoint(60, 30, 0);
    leftDisplay.eye->addPoint(50, 50, 0);
    leftDisplay.eye->addPoint(10, 50, 0);

    leftDisplay.iris->addPoint(20, 20, 0);
    leftDisplay.iris->addPoint(40, 30, 0);
    leftDisplay.iris->addPoint(20, 40, 0);

    // One Pixel wrong at this iris
    // leftDisplay.iris->addPoint(30, 20, 0);
    // leftDisplay.iris->addPoint(40, 30, 0);
    // leftDisplay.iris->addPoint(35, 40, 0);

    leftDisplay.iris->setAnimationDuration(2000);
    leftDisplay.eye->setAnimationDuration(2000);

    //leftDisplay.animate(50);
    //leftDisplay.drawEye();
    
    // ROS_INFO("%i", leftDisplay.width());
    // ROS_INFO("%i", leftDisplay.widthReal());
    // leftDisplay.drawPixel(7, 1, 1);
    // leftDisplay.drawPixel(8, 1, 1);

    // leftDisplay.drawPixel(1, 2, 1);
    // leftDisplay.drawPixel(7, 2, 1);
    // leftDisplay.drawPixel(8, 2, 1);

    // leftDisplay.drawPixel(1, 3, 1);
    // leftDisplay.drawPixel(7, 3, 1);
    // leftDisplay.drawPixel(8, 3, 1);

    // std::cout << "W: " << leftDisplay.width() << " H: " <<  leftDisplay.height() << " P: " << leftDisplay.getPixel(1, 1);

    // ROS_INFO("%i", leftDisplay.width());
    // ROS_INFO("%i", leftDisplay.height());
    // ROS_INFO("%i", leftDisplay.getPixel(1, 1));
    // ROS_INFO("%i", leftDisplay.getPixel(3, 1));

}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 * See http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29 
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "eyeSimulationNode");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Publisher redisPublisher = n.advertise<systemcore::RedisMessage>("message/redis/generic", 10);

  leftDisplay.linkGenericRosPublisher(&redisPublisher, "simulation/eyes/left");
  rightDisplay.linkGenericRosPublisher(&redisPublisher, "simulation/eyes/right");

  ros::Rate loop_rate(5);

  testEye();

  ros::Duration(1.5).sleep();

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {

  

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    systemcore::RedisMessage redisMsg;

    std::stringstream ssKey;
    ssKey << "simulation/eyes/left";
    
    redisMsg.key = ssKey.str();

    std::stringstream ssJson;
    ssJson << "{\"rows\": 128, \"cols\": 64, \"bitMap\": [255, 170, 85, 20, 123]}";

    redisMsg.json = ssJson.str();


    // std::stringstream ss;
    // ss << "hello world " << count;
    // msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());
    //ROS_INFO("%s", redisMsg.json.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //redisPublisher.publish(redisMsg);

    
    
    //ROS_INFO("%i", leftDisplay.getPixel(1, 1));

    leftDisplay.animate(200);
    leftDisplay.drawEye();

    leftDisplay.display();
    //rightDisplay.display();

    ros::spinOnce();

    
    loop_rate.sleep();
    

    
    ++count;

    if (count == 25) {
      test1();      
    }
    if (count == 50) {
      testEye();
      count = 0;
    }
  }


  return 0;
}