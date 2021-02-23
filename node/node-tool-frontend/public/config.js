/*
This module holds constants for easy access
*/

// URL of the API to control
export const API_URL = "http://" + location.hostname + "/node-tool-api";

// time between node list refreshes in milliseconds
export const NODE_LIST_REFRESH_COOLDOWN = 1000;

// time between log list refreshes in milliseconds
export const LOG_LIST_REFRESH_COOLDOWN = 500;

// for how many seconds to request logs for. (maximum age of displayed logs)
// note that setting this higher than how long the API keeps logs in memory will have no effect.
export const LOG_REQUEST_SECONDS = 300;