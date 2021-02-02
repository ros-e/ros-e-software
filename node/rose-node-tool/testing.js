/**
 * Script to test the node_tool
 * 
 * Johannes Sommerfeldt, 2021-02
 */

const NodeManager = require("./node_tool");

let nm = new NodeManager();

console.log(nm.listNodes());


nm.startNode("helloros_py", "talker");
nm.startNode("helloros_py", "listener");

setTimeout(() => {
    console.log(nm.listNodes());
}, 3000);

setTimeout(() => {
    nm.stopNode("helloros_py", "talker");
    nm.stopNode("helloros_py", "listener");
    console.log(nm.listNodes());
}, 5000);



