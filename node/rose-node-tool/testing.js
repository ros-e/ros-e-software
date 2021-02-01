/**
 * Script to test the node_tool
 * 
 * Johannes Sommerfeldt, 2021-01
 */

const NodeManager = require("./node_tool");
const fs = require("fs");
/*
let nm = new NodeManager();

nm.startNode("helloros_py", "talker");
nm.startNode("helloros_py", "listener");

setTimeout(() => {
    console.log(nm.listNodes());
}, 3000);

setTimeout(() => {
    nm.stopNode("helloros_py", "talker");
    nm.stopNode("helloros_py", "listener");
}, 5000);
*/

let softwareDirPath = __dirname + "/../..";
let nodeSrcPath = softwareDirPath + "/ros2/ros_ws/src";
console.log(fs.readdirSync(nodeSrcPath));