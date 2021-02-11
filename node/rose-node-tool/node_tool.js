/**
 * The node tool that manages ros2-nodes and so on.
 *
 * Johannes Sommerfeldt, 2021-02
 */

// import other files
const NodeHandle = require("./node_handle");
const AutostartConfiguration = require("./autostart");
const custom_errors = require("./custom_errors");
const config = require("./config");

// import external modules
const util = require("util");
const child_process = require("child_process");
const fs = require("fs");
const fsp = require("fs/promises");


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
    #activeAutostartConfig = new AutostartConfiguration(config.ACTIVE_AUTOSTART_FILE_PATH);

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
            throw new custom_errors.WrongStateError("Node is already running!");
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
            throw new custom_errors.WrongStateError("The node was not running!");
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
     * @returns {string} The log messages as JSON string
     */
    getLogs(packageName, nodeName, seconds) {

        let node = this.#findNode(packageName, nodeName);
        let logs = node.getLogs(seconds);
        return JSON.stringify(logs, null, 2);
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
        throw new custom_errors.InvalidArgumentError("Unknown combination of package and node names!");
    }

    // ===============================================================================
    // AUTOSTART

    /**
     * Starts all nodes listed in the currently active autostart configuration
     */
    async runAutostart() {
        let autostartEntries = await this.#activeAutostartConfig.getConfig();
        for (let entry of autostartEntries) {

            try {
            // start the node
            this.startNode(entry.packageName, entry.nodeName);

            // then wait the specified delay to let the node initialize
            await new Promise((resolve, reject) => setTimeout(() => resolve(), entry.delay));

            } catch (e) {
                console.error("Starting " + entry.packageName + " " + entry.nodeName + " was skipped: " + e.message);
            }
        }
    }

    /**
     * Gets a string representation of the current autostart configuration
     * @returns A JSON string of the autostart configuration
     */
    async getAutostart() {
        let autostartEntries = await this.#activeAutostartConfig.getConfig();
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
            throw new custom_errors.WrongStateError("Unknown node!");
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

    /**
     * Returns all saved autostart presets as a JSON object
     * @return {Promise<string>} JSON string representing a list of presets
     */
    async listAutostartPresets() {
        // create the directory in case it doesn't exist
        try {
            await fsp.mkdir(config.AUTOSTART_PRESETS_DIR_PATH);
            console.log("Created autostart preset directory.")
        } catch (e) {
            // nothing, since ideally the directory already exists anyway
        }

        let filesInDir = await fsp.readdir(config.AUTOSTART_PRESETS_DIR_PATH);
        let autostartFiles = filesInDir.filter((name) => name.endsWith(".json"));

        let presetList = new Object();
        for (let fileName of autostartFiles) {
            // cut off the .json at the end of the name
            let presetName = fileName.substring(0, fileName.length - 5);

            // get the preset data from the access object
            let presetDao = NodeTool.#getDaoForPreset(presetName);
            presetList[presetName] = await presetDao.getConfig();
        }
        return JSON.stringify(presetList, null, 2);
    }

    /**
     * Creates a new preset out of the current autostart configuration data
     * @param {string} presetName The name the preset should have. May only contain symbols that are allowed in file names.
     */
    async saveAsAutostartPreset(presetName) {
        // prepare access to the preset file
        let presetDAO = NodeTool.#getDaoForPreset(presetName);

        // write current config to the preset file
        let currentConfig = await this.#activeAutostartConfig.getConfig();
        await presetDAO.setConfig(currentConfig);
    }

    /**
     * Loads a preset, setting it as active autostart configuration
     * @param {string} presetName The name of the preset to load (case sensitive)
     */
    async loadAutostartPreset(presetName) {
        // prepare access to the preset file
        let presetDAO = NodeTool.#getDaoForPreset(presetName);

        if (await presetDAO.hasFile() === false) {
            throw new custom_errors.WrongStateError("No such preset: " + presetName + "!");
        }

        // write the preset config to the current config
        let presetConfig = await presetDAO.getConfig();
        await this.#activeAutostartConfig.setConfig(presetConfig);
    }

    /**
     * Removes a preset permanently
     * @param {string} presetName The name of the preset to delete (case sensitive)
     */
    async deleteAutostartPreset(presetName) {
        try {
            let preset = NodeTool.#getDaoForPreset(presetName);
            await preset.deleteFile();
        } catch (e) {
            throw new custom_errors.WrongStateError("The preset could not be deleted. It probably does not exist");
        }
    }

    /**
     * Gets the AutostartConfiguration access object for the specified preset name
     * @param {string} presetName Name of the preset to get the access object for
     * @return {AutostartConfiguration} The access object for the preset
     * @throws {InvalidArgumentError} When there is a '/' within the presetName
     */
    static #getDaoForPreset(presetName) {
        if (presetName.includes("/")) {
            throw new custom_errors.InvalidArgumentError("'/' is invalid in the preset name");
        }

        let filePath = config.AUTOSTART_PRESETS_DIR_PATH + presetName + ".json";

        return new AutostartConfiguration(filePath);
    }

    // ============================================================================
    // WORKSPACE AND PACKAGE BUILDING

    /**
     * Runs colcon build on the specified packages
     * @param {string[]} packageNames The names of the packages to build
     */
    async buildPackages(...packageNames) {

        // no point in building a package multiple times, since one build covers all its nodes
        let packageNameSet = new Set(packageNames);

        // validate package names
        for (let packageName of packageNameSet) {

            // check if package exists by seeing if at least 1 node is know for it
            let nodeForPackage = this.#nodes.find((node) => node.packageName === packageName);
            if (nodeForPackage == false) {
                throw new custom_errors.InvalidArgumentError("Unknown package '" + packageName + "'! All builds aborted.");
            }
        }

        // concatenate the names to they can be used in a single command
        let packagesString = Array.from(packageNameSet).join(" ");

        // build the package(s) using colcon build in the ros workspace
        let command = "cd " + config.ROS_WORKSPACE_PATH + " && colcon build --packages-select " + packagesString;

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

        return this.buildPackages(...packageNames);
    }

    // ==================================================================================
    // PRIVATE HELPER METHODS


    /**
     * Searches the file system for information synchronously (= using blocking IO!)
     * @returns {NodeHandle[]} A list of NodeHandle objects with the information found
     */
    static #assembleNodeListSync() {
        let foundNodes = [];

        // for each package in the workspace
        let packageNames = fs.readdirSync(config.SRC_PATH);
        for (let packageName of packageNames) {

            let packagePath = config.SRC_PATH + packageName;

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

                    // check whether build for this script is up-to-date
                    let srcFilePath = config.SRC_PATH + packageName + "/" + packageName + "/" + node.fileName;
                    let buildFilePath = config.BUILD_PATH + packageName + "/build/lib/" + packageName + "/" + node.fileName;
                    // get last modified time (floored to seconds, because build files seems to only have whole second precision)
                    let srcFileModDate = Math.floor(fs.statSync(srcFilePath).mtimeMs / 1000);
                    let buildFileModDate = Math.floor(fs.statSync(buildFilePath).mtimeMs / 1000);

                    let isUpToDate = buildFileModDate >= srcFileModDate;
                    node.isUpToDate = isUpToDate;

                    foundNodes.push(node);
                }

            } catch (e) {
                //console.log(e.name + ": " + e.message);
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

