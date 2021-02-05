/**
 * The node tool thta manages ros2-nodes and so on.
 *
 * Johannes Sommerfeldt, 2021-02
 */

const NodeHandle = require("./node_handle");
const AutostartConfiguration = require("./autostart");

const util = require("util");
const child_process = require("child_process");
const fs = require("fs");
const fsp = require("fs/promises");

const ACTIVE_AUTOSTART_FILE_PATH = __dirname + "/autostart.json";
const AUTOSTART_PRESETS_DIR_PATH = __dirname + "/autostart_presets/";

// workspace paths
const ROS_WORKSPACE_PATH = "/home/rose/software/ros2/ros_ws";
const SRC_PATH = ROS_WORKSPACE_PATH + "/src/";
const BUILD_PATH = ROS_WORKSPACE_PATH + "/build/";


/**
 * Keeps track of nodes
 */
module.exports = class NodeTool {

    /**
     * stores the NodeHandle objects for each node that was found in the workspace
     * @type {NodeHandle[]}
     */
    #nodes = [];

    // access object for the active autostart configuration
    #activeAutostartConfig = new AutostartConfiguration(ACTIVE_AUTOSTART_FILE_PATH);

    constructor() {
        this.#nodes = NodeTool.#assembleNodeListSync();
    }

    // ====================================================================
    // NODE MANAGEMENT

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
    async stopNode(packageName, nodeName) {

        let node = this.#findNode(packageName, nodeName);

        // can't stop nodes that are not running
        if (node.isRunning == false) {
            throw new Error("The node is was not running!");
        }

        await node.kill();
    }

    /**
     * Stops all nodes that are currently marked as running
     */
    async stopAll() {
        for (let node of this.#nodes) {
            if (node.isRunning) {
                await node.kill();
            }
        }
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

    // ===============================================================================
    // AUTOSTART

    /**
     * Starts all nodes listed in the currently active autostart configuration
     */
    async runAutostart() {
        let autostartEntries = await this.#activeAutostartConfig.list();
        for (let entry of autostartEntries) {

            // start the node
            this.startNode(entry.packageName, entry.nodeName);

            // then wait the specified delay to let the node initialize
            await new Promise((resolve, reject) => setTimeout(() => resolve(), entry.delay));
        }
    }

    /**
     * Gets a string representation of the current autostart configuration
     * @returns A JSON string of the autostart configuration
     */
    async getAutostart() {
        let autostartEntries = await this.#activeAutostartConfig.list();
        return JSON.stringify(autostartEntries, null, 2);
    }

    /**
     * Adds a node to the current autostart configuration
     * @param {string} packageName The name of the package the node is in
     * @param {string} nodeName The name of the node to add
     * @param {number} index An integer where to sort the node into the order of starting
     * @param {number} delayMs How many milliseconds to wait after starting the node to let it initialize
     */
    async addAutostart(packageName, nodeName, index, delayMs) {
        if (this.#findNode(packageName, nodeName) == false) {
            throw new Error("Unknown node!");
        }
        await this.#activeAutostartConfig.insert(packageName, nodeName, index, delayMs);
    }

    /**
     * Removes a node from the current autostart configuration
     * @param {number} index An integer specifying which index from the list to remove
     */
    async removeAutostart(index) {
        await this.#activeAutostartConfig.remove(index);
    }

    async listAutostartPresets() {
        // TODO
    }

    async saveAsAutostartPreset(presetName) {
        // TODO
    }

    async LoadAutostartPreset(presetName) {
        // TODO
    }

    // ============================================================================
    // WORKSPACE AND PACKAGE BUILDING

    /**
     * Runs colcon build on the specified packages
     * @param {Set<string>} packageNameList The names of the packages to build
     */
    async buildPackages(packageNameList) {

        // validate package names
        for (let packageName of packageNameList) {

            // check if package exists by seeing if at least 1 node is know for it
            let nodeForPackage = this.#nodes.find((node) => node.packageName === packageName);
            if (nodeForPackage == false) {
                throw new Error("Unknown package '" + packageName + "'!");
            }
        }

        let packagesString = Array.from(packageNameList).join(" ");

        // build the package(s) using colcon build in the ros workspace
        let command = "cd " + ROS_WORKSPACE_PATH + " && colcon build --packages-select " + packagesString;

        try {
            // promisify to make exec use await syntax rather than callbacks
            let exec = util.promisify(child_process.exec);
            let { stdout, stderr } = await exec(command);
            return stdout;
        } catch (e) {
            console.error(e);
            throw e;
        }
    }

    /**
     * Build the packages of all nodes that were found being not up-to-date
     */
    async buildAllRequiringBuild() {
        // check which nodes are not up to date (need build)
        let nodesRequiringBuild = this.#nodes.filter((node) => node.isUpToDate === false);

        // get the package they belong to since only full packages can be built
        let packageNames = nodesRequiringBuild.map((node) => { return node.packageName });

        // no point in building a package multiple times, since one build covers all its nodes
        let uniquePackageNames = new Set(packageNames);

        return this.buildPackages(uniquePackageNames);
    }

    // ==================================================================================
    // PRIVATE HELPER METHODS

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
     * Searches the file system for information synchronously (= using blocking IO!)
     * @returns {NodeHandle[]} A list of NodeHandle objects with the information found
     */
    static #assembleNodeListSync() {
        let foundNodes = [];

        // for each package in the workspace
        let packageNames = fs.readdirSync(SRC_PATH);
        for (let packageName of packageNames) {

            let packagePath = SRC_PATH + packageName;

            // check it is a directory
            let stats = fs.statSync(packagePath);
            if (stats.isDirectory() == false) {
                return;
            }

            // seek for a setup.py for python packages
            try {
                // read the setup.py in this package. This will throw if the file doesn't exist in a package.
                let setupData = fs.readFileSync(packagePath + "/setup.py");

                // isolate info about nodes
                let nodeList = NodeTool.#getNodeInfoFromSetupFile(setupData);

                for (let node of nodeList) {
                    console.log(node);

                    // check whether build for this script is up-to-date
                    let srcFilePath = SRC_PATH + packageName + "/" + packageName + "/" + node.fileName;
                    let buildFilePath = BUILD_PATH + packageName + "/build/lib/" + packageName + "/" + node.fileName;
                    // get last modified time (floored to seconds, because build files seems to only have whole second precision)
                    let srcFileModDate = Math.floor(fs.statSync(srcFilePath).mtimeMs / 1000);
                    let buildFileModDate = Math.floor(fs.statSync(buildFilePath).mtimeMs / 1000);

                    let isUpToDate = buildFileModDate >= srcFileModDate;
                    node.isUpToDate = isUpToDate;

                    foundNodes.push(node);
                }

            } catch (e) {
                console.log(e.name + ": " + e.message);
            }

            // TODO handle packages written in c

        } // end of for loop
        return foundNodes;
    }

    /**
     * Filters the info about ROS2-nodes out of a setup.py file
     * @param {Buffer} setupFileData The content of the setup file as Buffer
     * @return {NodeHandle[]} A list of Nodes found in the file
     */
    static #getNodeInfoFromSetupFile(setupFileData) {

        let foundNodes = [];

        // isolate the info about ROS-node scripts
        let shorterBuffer = setupFileData.subarray(setupFileData.indexOf("'console_scripts'"));
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

            // cut off the packageName and the ":main" to isolate the file name
            let packageSplit = scriptInfo.split(".");
            let packageName = packageSplit[0];
            let fileInfo = packageSplit[1].split(":")[0];
            let fileName = fileInfo + ".py";

            foundNodes.push(new NodeHandle(packageName, nodeName, fileName, null));
        }
        return foundNodes;
    }

}
