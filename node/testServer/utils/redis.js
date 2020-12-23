const redis = require("redis");

const subscriber = redis.createClient();
const publisher = redis.createClient();


// function publishToROS(pubTopic, jsonMsg) {
exports.publishToROS = (pubTopic, jsonMsg) => {

    publisher.publish(pubTopic, typeof(jsonMsg) == String ? jsonMsg : JSON.stringify(jsonMsg));
    
}











