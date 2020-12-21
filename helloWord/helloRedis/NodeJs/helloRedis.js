/**
 * Redis HelloWorld for Node.js (written in JavaScript)
 * Could be used in an electron app
 * 
 * See: https://github.com/NodeRedis/node_redis 
 */

// first install the required redis package 
// > npm install redis
var redis = require("redis");

// create Client connection
// can be used as template for other (subscribing) clients
var client = redis.createClient({
  host: "localhost"
  , port: 6379
  , db: 0
});

client.on("error", function (err) {
  console.log("Error " + err);
  client.quit();
});


/** #################################################################
 * GET and SET Redis values #########################################
 ################################################################# */

var val = "helloRedisTest";
red.set("helloRedis/nodejs", val.toString()); // for other values than strings, you should call the .toString() method. Redis can only store strings.

// In nodejs redis all requests are handled with callback functions 
red.get("helloRedis/nodejs", function (err, reply) {
  if (reply) {
    console.log("Got value for helloRedis/nodejs:" + reply.toString());
  }
});



/** #################################################################
 * client management for subscribing and publishing #################
 ################################################################# */

// stores all subscribing clients created with this module
var clientList = {};
// incrementing client id for 
var clientID = 0;

/**
 * Method to create a new client and add this client to the clientList
 */
function getNewClient() {
  // Duplicate client
  var c = client.duplicate();

  // Create object containing the new client an a new  client id
  var retObj = { client: c, id: clientID }
  clientList[clientID++] = c;

  return retObj;
}

/**
 * Close a client with the given id from the client list
 * @param {Number} id client id 
 */
function closeClient(id) {
  var c = clientList[id];
  if (c) {
    c.quit();
    c = null;
    delete clientList[id];
  }
}



/** #################################################################
 * EXPORT objects ###################################################
 ################################################################# */

module.exports = {

  /**
   * @returns a new redis client
   */
  getClient: function () {
    return client.duplicate();
  },

  /**
   * Subscribe to a specific redis key
   * @param {string} key Redis Key of topic you want to subscribe to 
   * @param {Function} callback callback method that is called as soon as a new value is published under the topic
   */
  subscribeKey: function (key, callback) {
    var c = getNewClient();
    var sub = c.client;
    sub.on("error", function (err) {
      console.log("Error " + err);
      closeClient(c.id);
    });

    sub.on("message", function (channel, message) {
      console.log("sub channel " + channel + ": " + message);
      callback(message);
    });

    sub.on("ready", function () {
      sub.subscribe(key);
    })
  },

  /**
   * Publish a value to a channel
   * @param {String} key key to publish to
   * @param {String} value value to be published
   */
  publish: function (key, value) {
    if (client.connected) {
      console.log("Published " + String(value) + " at " + key);
      client.publish(key, String(value));
    }
  },

  /**
   * Function to properly close RedisConnector
   */
  closeAll: function () {

    var keys = [];
    for (const [key, value] of Object.entries(clientList)) {
      keys.push(key);
    }

    keys.forEach(k => {
      closeClient(k);
    });
  }
}
