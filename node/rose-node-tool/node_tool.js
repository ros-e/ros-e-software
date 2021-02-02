/**
 * Tool to managing ros nodes
 *
 * Johannes Sommerfeldt, 2021-02
 */

const NodeHandle = require("./node_handle");
const fs = require("fs");

const ROS2_SRC_PATH = "/home/rose/software/ros2/ros_ws/src/";

/**
 * Keeps track of nodes
 */
module.exports = class NodeManager {

    // stores which nodes are already running
    // key is the concatenated package and node name, value is the Node object
    #activeNodes = new Map();

    #nodes = [];

    constructor() {
        this.#nodes = this.#assembleNodeList();
    }

    /**
     * Lists all known nodes and their info and status
     * @returns {string} The list of known nodes as JSON string
     */
    listNodes() {
        return JSON.stringify(this.#nodes, null, 2);
    }

    /**
     * Starts a node unless it is already running.
     * @param {string} packageName The name of the package the node is in
     * @param {string} nodeName The name of the node to start
     */
    startNode(packageName, nodeName) {

        let node = this.#findNode(packageName, nodeName);

        // don't start a node that is already running
        if (node.isRunning) {
            throw new Error("Node is already running!");
        } 

        // run the node and add it to the list
        node.run();
    }

    /**
     * Stops a node, or throws an error if it is not running.
     * @param {string} packageName The name of the package the node is in
     * @param {string} nodeName The name of the node to stop
     */
    stopNode(packageName, nodeName) {

        let node = this.#findNode(packageName, nodeName);

        // can't stop nodes that are not running
        if (node.isRunning == false) {
            throw new Error("The node is was not running!");
        }

        node.kill();
    }

    /**
     * Gets all received outputs from the specified node within the last <seconds> seconds.
     * @param {string} packageName The name of the package the node is in
     * @param {string} nodeName The name of the node
     * @param {number} seconds How many seconds to get messages for
     * @returns {string[]} The log messages as array
     */
    getLogs(packageName, nodeName, seconds) {

        let node = this.#findNode(packageName, nodeName);

        return node.getLogs(seconds);
    }

    /**
     * Gets a node object from the list of known nodes
     * @param {string} packageName The name of the package the node is in
     * @param {string} nodeName The name of the node to find
     * @returns {NodeHandle} A NodeHandle object for the node. Throws an Error if none was found
     * @throws Error when no node was found for that packageName and nodeName
     */
    #findNode(packageName, nodeName) {
        for (let current of this.#nodes) {
            if (current.packageName === packageName && current.nodeName === nodeName) {
                return current;
            }
        }
        throw new Error("Unknown combination of package and node names!");
    }

    /**
     * Searches the file system for information 
     * @returns {NodeHandle[]} A list of NodeHandle objects with the information found
     */
    #assembleNodeList() {
        let foundNodes = [];

        // for each package in the workspace
        let packageNames = fs.readdirSync(ROS2_SRC_PATH)
        for (let packageName of packageNames) {

            let packagePath = ROS2_SRC_PATH + packageName;

            // check it is a directory
            let stats = fs.statSync(packagePath);
            if (stats.isDirectory() == false) {
                return;
            }

            // seek for a setup.py for python packages
            try {
                // read the setup.py in this package. This will throw if the file doesn't exist for a package.
                let setupData = fs.readFileSync(packagePath + "/setup.py");

                // isolate the info about ROS-node scripts
                let shorterBuffer = setupData.subarray(setupData.indexOf("'console_scripts'"));
                let openingBracketsIndex = shorterBuffer.indexOf("[");
                let closingBracketsIndex = shorterBuffer.indexOf("]");
                let nodeInfoString = shorterBuffer.subarray(openingBracketsIndex + 1, closingBracketsIndex).toString();

                // put node info into a list and clean up entries
                let nodeInfoList = nodeInfoString.split(",")
                    // remove white space and apostrophes
                    .map(entry => entry.trim().split("'").join(""))
                    // remove empty entry
                    .filter(entry => entry !== "");

                // disassemble the info
                for (let entry of nodeInfoList) {
                    let parts = entry.split(" = ");
                    let nodeName = parts[0];
                    let scriptInfo = parts[1];

                    // TODO up-to-date check
                    foundNodes.push(new NodeHandle(packageName, nodeName, scriptInfo, true));
                }

                console.log(packageName + ": ");
                console.log(nodeInfoList);

            } catch (error) {
                console.log(error.name + ": " + error.message);
            }

            // TODO handle packages written in c

        } // end of for loop
        return foundNodes;
    }
}
