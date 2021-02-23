"use strict";
/*
Main module that handles user interaction
*/

import * as ApiRequests from "./api_requests.js";
import { LOG_LIST_REFRESH_COOLDOWN, LOG_REQUEST_SECONDS, NODE_LIST_REFRESH_COOLDOWN } from "./config.js";

// variables for quick access to important DOM elements
const nodeListContainer = document.getElementById("node-list-container");
const nodeListEntryTemplate = document.getElementById("node-list-entry-template");
const logContainer = document.getElementById("log-container");
const logField = document.getElementById("log-field");
const logPauseToggleButton = document.getElementById("log-pause-toggle-button");

// add button event handlers
addDisablerButtonHandler(document.getElementById("build-all-button"), ApiRequests.postBuildAll);
addDisablerButtonHandler(document.getElementById("build-modified-button"), ApiRequests.postBuildModified);
logPauseToggleButton.addEventListener("click", () => startLogging(selectedNode.packageName, selectedNode.nodeName));


// Stores the latest requested node list. Used to check for changes in the list.
let latestNodeListResponse = "";

// The node currently selected in the list and whose log messages to show
const selectedNode = {
    packageName: "",
    nodeName: ""
}

// keep the node list up-to-date by refreshing it regularly
refreshNodeListPeriodically();

// =========================================================================
// FUNCTIONS

/**
 * Can be awaited to delay the following code. This doesn't block the thread, only the async method it is called in.
 * @param {number} durationMilliseconds The duration to sleep in milliseconds
 */
async function delay(durationMilliseconds) {
    await new Promise((resolve) => setTimeout(resolve, durationMilliseconds));
}

/**
 * Requests the log messages for the specified node regularly and updates the HTML accordingly.
 * DO NOT await since this might loop for a long time.
 * Returns when the selected node changes or when the pause button is pressed.
 * @param {string} packageName The package name of the node to show the logs for
 * @param {string} nodeName The node name of the node to show the logs for
 */
async function startLogging(packageName, nodeName) {

    while (true) {
        // stop logging when the pause button is toggled to active or when the selected node changes
        let isPaused = logPauseToggleButton.classList.contains("active");
        let hasSelectionChanged = packageName !== selectedNode.packageName || nodeName !== selectedNode.nodeName;
        if (isPaused || hasSelectionChanged) {
            break;
        }

        let response = await ApiRequests.getLog(packageName, nodeName, LOG_REQUEST_SECONDS);
        let logs = await response.json();
        /** @type {string} */
        let logText = logs.join(" ");

        // put a default info text if there are no logs
        if (logText.trim() === "") {
            logText = "<Keine Logs verfügbar>";
        }

        // set the text to the log field
        logField.innerText = logText;

        // scroll text field to bottom
        logContainer.scrollTop = logContainer.scrollHeight;

        // wait before next request
        await delay(LOG_LIST_REFRESH_COOLDOWN);
    }
}

/**
 * Start to refresh the node list regularly. 
 * DO NOT await this funtion since it loops forever and will never return.
 */
async function refreshNodeListPeriodically() {
    while (true) {
        refreshNodeList();
        // wait before next refresh
        await delay(NODE_LIST_REFRESH_COOLDOWN);
    }
}

/**
 * Requests the node list and updates the displayed list if anything changed
 */
async function refreshNodeList() {
    // get list from API
    let response = await ApiRequests.getList();
    let responseBody = await response.text();

    // only rebuild the html when there actually was a change
    if (responseBody === latestNodeListResponse) {
        return;
    }
    latestNodeListResponse = responseBody;

    let nodes = JSON.parse(responseBody);

    // clear html list
    nodeListContainer.innerHTML = "";

    // convert each object from the node list to an HTML Element and append
    let newElement = buildNodeListHtml(nodes);
    nodeListContainer.appendChild(newElement);

    // if the previously selected node is still in the list, show it in the view
    expandSelectedEntryOnly();
}

/**
 * Builds a list of HTML elements out of a list of nodes
 * @param {any[]} nodes The list of node information as provided by the api
 */
function buildNodeListHtml(nodes) {

    let nodeListEntryElements = new DocumentFragment();

    for (let i = 0; i < nodes.length; i += 1) {
        let nodeInfo = nodes[i];

        // create an Element using the template
        /** @type {DocumentFragment} */
        let templateCopy = nodeListEntryTemplate.content.cloneNode(true);

        // find the html elements in the template 
        let entry = templateCopy.querySelector(".node-list-entry");
        let packageNameElement = templateCopy.querySelector(".package-name-p");
        let nodeNameElement = templateCopy.querySelector(".node-name-p");
        let fileNameElement = templateCopy.querySelector(".file-name-p");
        let buildIcon = templateCopy.querySelector(".build-warning-icon");
        let startButton = templateCopy.querySelector(".start-button");
        let stopButton = templateCopy.querySelector(".stop-button");
        let buildButton = templateCopy.querySelector(".build-button");

        // text elements with data
        packageNameElement.innerText = nodeInfo.packageName;
        nodeNameElement.innerText = nodeInfo.nodeName;
        fileNameElement.innerText = nodeInfo.fileName;

        // hide the build warning icon when the node is up to date
        if (nodeInfo.isUpToDate) {
            buildIcon.setAttribute("hidden", "");
        }

        // hide the button that makes no sense in current state
        if (nodeInfo.isRunning) {
            startButton.setAttribute("hidden", "");
        } else {
            stopButton.setAttribute("hidden", "");
        }

        // add event handler on the whole entry to make the node list entries collapsible
        entry.addEventListener("click", () => selectEntry(entry));

        // add event handlers for the buttons
        addDisablerButtonHandler(startButton, async () => {
            await ApiRequests.postStart(nodeInfo.packageName, nodeInfo.nodeName);
            refreshNodeList();
        });
        addDisablerButtonHandler(stopButton, async () => {
            await ApiRequests.postStop(nodeInfo.packageName, nodeInfo.nodeName);
            refreshNodeList();
        });
        addDisablerButtonHandler(buildButton, async () => await ApiRequests.postBuild(nodeInfo.packageName, nodeInfo.nodeName));

        nodeListEntryElements.appendChild(templateCopy);
    }
    return nodeListEntryElements;
}

/**
 * Sets the clicked entry of the node list as selected
 * @param {Element} clickedNodeListEntry The node list entry that was clicked
 */
function selectEntry(clickedNodeListEntry) {
    // find out what the packageName and nodeName of this entry are
    let { packageName, nodeName } = getPackageAndNodeName(clickedNodeListEntry);

    // update the variable tracking which is selected
    selectedNode.packageName = packageName;
    selectedNode.nodeName = nodeName;

    // update the view
    expandSelectedEntryOnly();

    // set logging to the selected node
    startLogging(packageName, nodeName);
}

/**
 * Gets the packageName and nodeName of this node-list-entry Element
 * @param {Element} nodeListEntry A node-list-entry Element
 * @returns {{packageName: string, nodeName: string}} packageName and nodeName of the specified entry
 */
function getPackageAndNodeName(nodeListEntry) {
    let packageName = nodeListEntry.querySelector(".package-name-p").innerText;
    let nodeName = nodeListEntry.querySelector(".node-name-p").innerText;
    return { packageName: packageName, nodeName: nodeName };
}

/**
 * Collapses all elements in the node list except the selected one, which will be expanded
 */
function expandSelectedEntryOnly() {

    // reveal extra row for this entry and hide it for all other node list entries
    for (let entry of nodeListContainer.children) {

        let { packageName, nodeName } = getPackageAndNodeName(entry);
        let extraRow = entry.querySelector(".node-list-entry-extra");

        if (packageName === selectedNode.packageName && nodeName === selectedNode.nodeName) {
            extraRow.toggleAttribute("hidden", false);
            entry.classList.add("bg-light");
        } else {
            extraRow.toggleAttribute("hidden", true);
            entry.classList.remove("bg-light");
        }
    }
}

/**
 * Adds a click-handler to the specified button that will be disabled while the specified function is awaiting
 * @param {HTMLButtonElement} button The button to add the on-click handler for
 * @param {Function} functionToWrap The function to call within the handler
 */
async function addDisablerButtonHandler(button, functionToWrap) {
    button.addEventListener("click", async () => {
        button.toggleAttribute("disabled", true);
        await functionToWrap();
        button.toggleAttribute("disabled", false);
    });
}