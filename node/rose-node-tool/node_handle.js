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

const child_process = require("child_process");
const winston = require("winston");

/**
 * Class for easy control over ROS2-Node's processes
 */
module.exports = class NodeHandle {

    // private variables
    #packageName;
    #nodeName;
    #processHandle = null;
    #logs = new Map();
    #logger;

    /**
     * Creates a Node object used to manage a node
     * @param {string} packageName The name of the package the node is in
     * @param {string} nodeName The name of the node
     */
    constructor(packageName, nodeName) {
        this.#packageName = packageName;
        this.#nodeName = nodeName;
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
     * @returns The process handle of the spawned process
     */
    run() {

        // make sure to only spawn one process of the same node
        if (this.#processHandle != null) {
            throw new Error("This node is already running!");
        }

        // spawn the node detached, to make it the leader of its process group (make pid == pgid)
        let child = child_process.spawn("ros2", ["run", this.#packageName, this.#nodeName], { detached: true });

        console.log("Spawned " + this.#packageName + " " + this.#nodeName + " with PID: " + child.pid);

        // add status listeners
        child.on("spawn", () => console.log("Child was spawned"));
        child.on("exit", (code, signal) => console.log("Child exited with code " + code + " from signal " + signal));
        child.on("close", (code, signal) => console.log("Child closed all stdio with code " + code + " from signal " + signal));

        // add stdio pipe listeners
        child.stdout.on("data", (data) => {
            this.#handleLogs(data);
        });
        child.stderr.on("data", (data) => {
            this.#handleLogs(data);
        });

        this.#processHandle = child;
    }

    /**
     * Sends a termination signal to the node
     */
    kill() {
        // make sure to only kill processes that have been spawned before
        if (this.#processHandle == null) {
            throw new Error("This node is not currently running!");
        }

        let pid = this.#processHandle.pid;

        // kill all processes in the group id, since the ros2 run spawns another child
        // needs bash as shell. Using the default /bin/sh resulted in "kill: illegial option -S" error
        child_process.exec("kill -SIGINT -" + pid, { shell: "/bin/bash" }, (error, stdout, stderr) => {
            console.log("Sent kill to: " + this.#packageName + " " + this.#nodeName + " with PID: " + pid);
            // TODO error handling
        });

        // delete the process handle so another kill() will fail
        this.#processHandle = null;
    }

    /**
     * Returns log data for this node for the time window
     * @param {number} seconds The last few seconds to return log data for. For example, a value of 1 means 
     * only return the log data that came within the last second.
     * @returns the log messages from the node within the time window 
     */
    getLog(seconds) {
        let returnData = [];
        let now = Date.now();
        let startTime = now - 1000 * seconds;

        // filter for data that came within the specified time window
        for (let [logTime, data] in this.#logs.entries()) {
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
        let nowDate = (new Date()).toLocaleString("de-DE");
        this.#logger.info(nowDate + ": " + data);

        // debug:
        console.log(nowDate + ": " + data);
    }
}