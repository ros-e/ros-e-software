/**
 * Code dealing with processes of ROS2-nodes
 * 
 * Johannes Sommerfeldt, 2021-02
 */

// minimim time a log message will be kept in memory. (milliseconds)
const LOG_TIMEOUT_MS = 60 * 1000;

// maximum hard drive space occupied by log files per node. (Bytes)
const LOG_FILE_SIZE_LIMIT = 10 ** 6
// how many files to split the lig per node into. (min 2). More files => smaller files
const LOG_FILE_COUNT = 2;

// path to write log files to
const LOG_DIRECTORY_PATH = __dirname + "/logs/";

const custom_errors = require("./custom_errors");

const child_process = require("child_process");
const winston = require("winston");

/**
 * Access object for easy control over ROS2-Node's processes
 */
module.exports = class NodeHandle {

    // Node data
    packageName;
    nodeName;
    fileName;
    isUpToDate;
    isRunning = false;  // somewhat redundant with #processHandle, but the handle should be private

    // private variables
    #processHandle = null;
    #logs = new Map();
    #logger;

    constructor(packageName, nodeName, fileName, isUpToDate) {
        this.packageName = packageName;
        this.nodeName = nodeName;
        this.fileName = fileName;
        this.isUpToDate = isUpToDate;
        this.#logger = winston.createLogger({
            transports: [new winston.transports.File({
                filename: LOG_DIRECTORY_PATH + packageName + "_" + nodeName + ".log",
                maxsize: LOG_FILE_SIZE_LIMIT / LOG_FILE_COUNT,
                maxFiles: LOG_FILE_COUNT,
                tailable: true
            })],
            format: winston.format.simple()
        });
    }

    /**
     * Spawns the node as a new process
     */
    run() {

        // make sure to only spawn one process of the same node
        if (this.#processHandle != null) {
            throw new custom_errors.WrongStateError("This node is already running!");
        }

        // run setup first so the ros2 commands can be added to the environment
        let command = "source ~/software/ros2/ros_ws/install/setup.bash";
        command += " && ros2 run";

        // spawn the node detached, to make it the leader of its process group (make pid == pgid)
        let child = child_process.spawn(command, [this.packageName, this.nodeName], { shell: "/bin/bash", detached: true });

        console.log("Spawned " + this.packageName + " " + this.nodeName + " with PID: " + child.pid);

        // add status listeners
        child.on("spawn", () => console.log("Child was spawned"));
        child.on("exit", (code, signal) => {
            console.log(this.packageName + " " + this.nodeName + " exited with code " + code + " from signal " + signal)
        });
        child.on("close", (code, signal) => {
            console.log(this.packageName + " " + this.nodeName + " closed all stdio with code " + code + " from signal " + signal)
        });
        child.on("error", (err) => {
            // TODO reject this promise / throw error (?)
        });

        // add stdio pipe listeners
        child.stdout.on("data", (data) => {
            this.#handleLogs(data);
        });
        child.stderr.on("data", (data) => {
            this.#handleLogs(data);
        });

        // update object
        this.#processHandle = child;
        this.isRunning = true;
    }

    /**
     * Sends a termination signal to the node
     */
    async kill() {
        // make sure to only kill processes that have been spawned before
        if (this.#processHandle == null) {
            throw new custom_errors.WrongStateError("This node is not currently running!");
        }

        let pid = this.#processHandle.pid;

        // execute the actual shell command in a promise to make it awaitable
        await new Promise((resolve, reject) => {

            // kill all processes in the group id, since the ros2 run spawns another child
            // needs bash as shell. Using the default /bin/sh resulted in "kill: illegial option -S" error
            child_process.exec("kill -SIGINT -" + pid, { shell: "/bin/bash" }, (error, stdout, stderr) => {
                console.log("Sent kill to: " + this.packageName + " " + this.nodeName + " with PID: " + pid);
                if (error) {
                    reject(error);
                } else {
                    resolve();
                }
            });
        });

        // delete the process handle so another kill() will fail
        this.#processHandle = null;
        this.isRunning = false;
    }

    /**
     * Returns log data for this node for the time window
     * @param {number} seconds The last few seconds to return log data for. For example, a value of 1 means 
     * only return the log data that came within the last second.
     * @returns {string[]} the log messages from the node within the time window 
     */
    getLogs(seconds) {
        let returnData = [];
        let now = Date.now();
        let startTime = now - 1000 * seconds;

        // filter for data that came within the specified time window
        for (let [logTime, data] of this.#logs.entries()) {
            if (logTime > startTime) {
                returnData.push(data);
            }
        }

        // return all matching data
        return returnData;
    }

    /**
     * Callback for when child process produce output
     * @param {string} data The output data from the child process
     */
    #handleLogs(data) {

        // store log temporarily in map
        let nowMillis = Date.now();
        this.#logs.set(nowMillis, data);

        // delete logs that are too old
        setTimeout(() => {
            this.#logs.delete(nowMillis);
        }, LOG_TIMEOUT_MS);

        // store in file
        let nowDate = (new Date()).toLocaleString("de-DE", { timeZone: "Europe/Berlin" });
        this.#logger.info(nowDate + ": " + data);

        // debug:
        //console.log(nowDate + ": " + data);
    }
}
