/**
 * Script to test the node_tool
 * 
 * Johannes Sommerfeldt, 2021-02
 */

const NodeManager = require("./node_tool");

let nm = new NodeManager();


nm.getAutostart()
    .then((data) => console.log(data))
    .then(() => nm.addAutostart("helloros_py", "talker", 0, 500))


/*
nm.startNode("helloros_py", "talker");
nm.startNode("helloros_py", "listener");

setTimeout(() => {
    console.log(nm.listNodes());
}, 3000);

setTimeout(() => {
    console.log("Logs:\n" + nm.getLogs("helloros_py", "listener", 1));
}, 4000);

setTimeout(() => {
    nm.stopNode("helloros_py", "talker");
    nm.stopNode("helloros_py", "listener");
    //console.log(nm.listNodes());
}, 5000);
*/


