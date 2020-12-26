const redis = require("redis");

const client = redis.createClient();
const subscriber = redis.createClient();
const publisher = redis.createClient();


// function publishToROS(pubTopic, jsonMsg) {
exports.publishToROS = (pubTopic, jsonMsg) => {

    publisher.publish(pubTopic, typeof(jsonMsg) == String ? jsonMsg : JSON.stringify(jsonMsg));
    
}

exports.client = client;

// exports.getSync = (key) => {
//     return key != undefined ? client.get(key) : null;
// }

// exports.setKey = (key, value) => {
//     return key != undefined ? client.get(key) : null;
// }








