/**
 * Script to test the node_tool without a web-API
 * 
 * Johannes Sommerfeldt, 2021-02
 */

"use strict";


const NodeTool = require("./node_tool");
const config = require("./config");

let nodeTool = new NodeTool();
console.log("constructor done");

(async () => {

    refreshPeriodically();

    await new Promise(resolve => setTimeout(resolve, 6000));

    nodeTool = null;
    console.log("node tool was nulled")

    // let before = Date.now();
    // await nodeTool.init();
    // let after = Date.now();
    // console.log(after-before);
    // console.log("init 1 done:");
    // console.log(nodeTool.listNodes());

})();


async function refreshPeriodically() {
    while (nodeTool) {
        try {
            let before = Date.now();            // debug
            await nodeTool.refreshNodeList();
            let after = Date.now();             // debug
            console.log(after - before);        // debug
        } catch (e) {
            console.error("Error while refreshing: " + e.message);
        }
        // sleep
        await new Promise((resolve) => setTimeout(resolve, config.NODE_LIST_REFRESH_COOLDOWN));
    }
}
