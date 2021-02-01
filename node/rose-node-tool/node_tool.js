/**
 * Tool to managing ros nodes
 *
 * Johannes Sommerfeldt, 2021-02
 */

const NodeHandle = require("./node_handle");

/**
 * Keeps track of node
 */
module.exports = class NodeManager {

    // stores which nodes are already running
    // key is the concatenated package and node name, value is the Node object
    #activeNodes = new Map();

    /**
     * Lists all known nodes and their status
     */
    listNodes() {
        // TODO find all nodes that exist but are not active

        return this.#activeNodes.keys();
    }

    /**
     * Starts a node unless it is already running.
     * @param {string} packageName The name of the package the node is in
     * @param {string} nodeName The name of the node to start
     */
    startNode(packageName, nodeName) {
        let key = NodeManager.#joinStrings(packageName, nodeName);

        // don't start a node that is already running
        if (this.#activeNodes.has(key)) {
            throw new Error("Node is already running!");
        }

        // create a Node object
        let node = new NodeHandle(packageName, nodeName);

        // run the node and add it to the list
        node.run();
        this.#activeNodes.set(key, node);
    }

    /**
     * Stops a node, or throws an error if it is not running.
     * @param {string} packageName The name of the package the node is in
     * @param {string} nodeName The name of the node to start
     */
    stopNode(packageName, nodeName) {
        let key = NodeManager.#joinStrings(packageName, nodeName);

        // can't stop nodes that are not running
        if (this.#activeNodes.has(key) == false) {
            throw new Error("The node is was not running!");
        }
        let node = this.#activeNodes.get(key);
        node.kill();
    }

    /**
     * Gets all received outputs from the specified node within the last <seconds> seconds.
     * @param {string} packageName The name of the package the node is in
     * @param {string} nodeName The name of the node to start
     * @param {number} seconds How many seconds to get messages for
     */
    getLogs(packageName, nodeName, seconds) {
        let key = NodeManager.#joinStrings(packageName, nodeName);

        if (this.#activeNodes.has(key) == false) {
            throw new Error("No such node is active yet!");
        }

        let node = this.#activeNodes.get(key);
        return node.getLog(seconds);

    }

    /**
     * Makes one string out of 2 strings. This method is for avoiding reduntant code.
     * @param {string} str1 
     * @param {string} str2 
     * @returns a string consisting of the inputs joined over a white space
     */
    static #joinStrings(str1, str2) {
        return str1 + " " + str2;
    }
}
