/**
 * Script to test the node_tool without a web-API
 * 
 * Johannes Sommerfeldt, 2021-02
 */

const NodeTool = require("./node_tool");

let nm = new NodeTool();

// console.log(process.env);


(async () => {
  
    // console.log("before");
    // console.log(await nm.getAutostart());

    // await nm.addAutostart("helloros_py", "talker", 1, 500);

    // console.log("between");
    // console.log(await nm.getAutostart());

    // await nm.removeAutostart(0);

    // await new Promise((resolve, reject) => setTimeout(resolve, 2000));

    // console.log("after");

    // await nm.removeAutostart();
    // console.log(await nm.getAutostart());

    console.log(await nm.listAutostartPresets());

    await nm.deleteAutostartPreset("test_preset");

    console.log(await nm.listAutostartPresets());

    // // await nm.saveAsAutostartPreset("test_preset");
    // // console.log(await nm.listAutostartPresets());

    // await nm.loadAutostartPreset("test_preset");

    // console.log(await nm.getAutostart());


    // await nm.runAutostart();
    // console.log(nm.listNodes());

    // console.log(await nm.buildAllRequiringBuild());

    // await new Promise((resolve, reject) => setTimeout(resolve, 2000));

    // await nm.addAutostart("helloros_py", "talker", 1, 1000);
    // console.log(await nm.getAutostart());

    // console.log("---------------------------------------------------------");

    // await nm.removeAutostart(0);
    // console.log(await nm.getAutostart());

    // await nm.stopAll();
    // // console.log(nm.listNodes());
})();


/*
nm.getAutostart()
    .then((data) => console.log(data))

    .then(() => nm.addAutostart("helloros_py", "listener", 1, 500))

    .then(() => console.log("between: "))
    .then(() => nm.getAutostart())
    .then((data) => console.log(data))

    .then(() => nm.removeAutostart(0))

    .then(() => console.log("after: "))
    .then(() => nm.getAutostart())
    .then((data) => console.log(data))
*/

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


