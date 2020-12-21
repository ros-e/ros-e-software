

#ifndef _ROSE_DISPLAY_PUBLISHER_H
#define _ROSE_DISPLAY_PUBLISHER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "systemcore/RedisMessage.h"
#include <ros/console.h>

#include <string>
#include "GFX_Buffer.h"


/*!
    @brief  Class that stores state and functions for interacting with
            Rose_DisplayPublisher simulation displays.
*/
class Rose_DisplayPublisher : public GFX_Buffer
{
public:
    Rose_DisplayPublisher(uint8_t w, uint8_t h);

    ~Rose_DisplayPublisher(void);

    ros::Publisher * redisPublisher;
    std::string redisTopic;

    void linkGenericRosPublisher(ros::Publisher * redisPublisher, std::string redisTopic);

    void display();
    void clearDisplay(void);

    uint8_t *getBuffer(void);

private:
};

#endif // _ROSE_DISPLAY_PUBLISHER_H
