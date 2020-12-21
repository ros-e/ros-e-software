

#include <sstream>

#include "Rose_DisplayPublisher.h"

Rose_DisplayPublisher::Rose_DisplayPublisher(uint8_t w, uint8_t h) : GFX_Buffer(h, w)
{
}


/*!
    @brief  Destructor for Rose_DisplayPublisher object.
*/
Rose_DisplayPublisher::~Rose_DisplayPublisher(void) {
    delete(this);
}

uint8_t *Rose_DisplayPublisher::getBuffer(void) {
  return buffer;
}

void Rose_DisplayPublisher::display() {
    

    systemcore::RedisMessage redisMsg;

    // std::stringstream ssKey;
    // ssKey << "simulation/eyes/left";
    
    redisMsg.key = this->redisTopic;

    std::stringstream ssJson;

    ssJson << "{\"rows\": 128, \"cols\": 64, \"bitMap\": [";

    int colsInBuffer = (this->width() + 7) / 8;

    for (int r = 0; r < this->height(); r++ ) {
        for (int c = 0; c < colsInBuffer; c++ ) {
            
            if (r > 0 || c > 0) ssJson << ", ";

            int index = r * colsInBuffer + c;
            //ROS_INFO("[%i, %i]: %i %i", c * colsInBuffer, r, index, this->buffer[index]);

            ssJson << std::to_string(this->buffer[index]);

        }   
    }
    

/*
    for (int i = 0; i < this->width() * ((this->height() + 7) / 8); i++)
    {
        if (i != 0) ssJson << ", ";
        //this->getPixel()
        ssJson << std::to_string(this->buffer[i]);
    }*/
    
    ssJson << "]}";

    redisMsg.json = ssJson.str();

    this->redisPublisher->publish(redisMsg);


}


void Rose_DisplayPublisher::linkGenericRosPublisher(ros::Publisher * redisPublisher, std::string redisTopic) {
    this->redisPublisher = redisPublisher;
    this->redisTopic = redisTopic;

}