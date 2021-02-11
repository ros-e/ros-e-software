/**
 * File that allows quick and easy definitions of constant values for the node tool and API
 * 
 * Johannes Sommerfeldt, 2021-01
 */

/**
 * Object that contains all global constants
 */
const config = {};


/* CRITICAL CONSTANTS */

// workspace paths
config.ROS_WORKSPACE_PATH = "/home/rose/software/ros2/ros_ws";
config.SRC_PATH = config.ROS_WORKSPACE_PATH + "/src/";
config.BUILD_PATH = config.ROS_WORKSPACE_PATH + "/build/";

// path to the ROS2 environment setup script 
config.ROS2_SETUP_SCRIPT_PATH = "/home/rose/software/ros2/ros_ws/install/setup.bash";


/* PREFERENCES */

// path to write log files to
config.LOG_DIRECTORY_PATH = __dirname + "/logs/";

// file paths for autostart configurations
config.ACTIVE_AUTOSTART_FILE_PATH = __dirname + "/autostart.json";
config.AUTOSTART_PRESETS_DIR_PATH = __dirname + "/autostart_presets/";

// logging preferences
config.LOG_TIMEOUT_MS = 60 * 1000;   // time a log message will be kept in memory. (milliseconds)
config.LOG_FILE_SIZE_LIMIT = 10 ** 6 // maximum hard drive space occupied by log files per node. (Bytes)
config.LOG_FILE_COUNT = 2;           // how many files to split the log per node into. (min 2). More files => smaller files

// server preferences
config.SERVER_PORT = 42750           // chosen arbitrarily, feel free to change


// make the object immutable and export it so other modules can use it as a container of constants
module.exports = Object.freeze(config);