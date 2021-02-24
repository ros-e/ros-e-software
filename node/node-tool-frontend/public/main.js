"use strict";
/*
Main module that handles user interaction
*/

import * as ApiRequests from "./api_requests.js";
import { LOG_LIST_REFRESH_COOLDOWN, LOG_REQUEST_SECONDS, NODE_LIST_REFRESH_COOLDOWN } from "./config.js";

// variables for quick access to static DOM elements
const nodeListContainer = document.getElementById("node-list-container");
const nodeListEntryTemplate = document.getElementById("node-list-entry-template");
const buildAllButton = document.getElementById("build-all-button");
const buildModifiedButton = document.getElementById("build-modified-button");
const logContainer = document.getElementById("log-container");
const logField = document.getElementById("log-field");
const logPauseToggleButton = document.getElementById("log-pause-toggle-button");
const autostartEntryTemplate = document.getElementById("autostart-entry-template");
const autostartTableBody = document.getElementById("autostart-table-body");
const autostartAddOkButton = document.getElementById("add-autostart-ok-button");
const autostartRemoveOkButton = document.getElementById("remove-autostart-ok-button");
const autostartSaveOkButton = document.getElementById("save-autostart-ok-button");
const presetEntryTemplate = document.getElementById("preset-entry-template");
const presetContainer = document.getElementById("preset-container");
const viewPresetModalHeading = document.getElementById("view-preset-modal-heading");
const viewPresetModalContentField = document.getElementById("view-preset-modal-content-p");

/* STATIC BUTTON EVENT HANDLERS */

// build buttons
addDisablerButtonHandler(buildAllButton, ApiRequests.postBuildAll);
addDisablerButtonHandler(buildModifiedButton, ApiRequests.postBuildModified);

// pause logging
// logPauseToggleButton.addEventListener("click", () => refreshLogPeriodically(selectedNode.packageName, selectedNode.nodeName));

// autostart modal confirm buttons
addDisablerButtonHandler(autostartAddOkButton, async function () {
    let packageName = document.getElementById("add-autostart-package-name-input").value;
    let nodeName = document.getElementById("add-autostart-node-name-input").value;
    let delay = document.getElementById("add-autostart-delay-input").value;
    let index = document.getElementById("add-autostart-index-input").value;

    // send request
    await ApiRequests.putAutostart(packageName, nodeName, delay, index);
    // refresh displayed list
    await loadAutostart();
});
addDisablerButtonHandler(autostartRemoveOkButton, async function () {
    let index = document.getElementById("remove-autostart-index-input").value;
    await ApiRequests.deleteAutostart(index);
    await loadAutostart();
});
addDisablerButtonHandler(autostartSaveOkButton, async function () {
    let presetName = document.getElementById("save-autostart-preset-name-input").value;
    await ApiRequests.putAutostartPresets(presetName);
    await loadPresets();
})


// Stores the latest requested node list. Used to check for changes in the list.
let latestNodeListResponse = "";

// stores parsed node list info to check content on demand
let nodeList = [];

// Stores the current preset info to check content on demand
let presets = {};

// The node currently selected in the list and whose log messages to show
const selectedNode = {
    packageName: "",
    nodeName: ""
}


// init the lists (and only update when user inputs require refreshes)
loadNodeList();
loadAutostart();
loadPresets();

refreshLogPeriodically();

// keep the node list up-to-date by refreshing it regularly
// refreshNodeListPeriodically();

// =========================================================================
// FUNCTIONS

/**
 * Requests the autostart configuration from the API and displays the content
 */
async function loadAutostart() {
    // get JSON from API
    let response = await ApiRequests.getAutostart();
    let autostartList = await response.json();

    let autostartElements = new DocumentFragment();

    for (let i = 0; i < autostartList.length; i += 1) {
        // create a new row for the table using the template
        let templateCopy = autostartEntryTemplate.content.cloneNode(true);

        let indexElement = templateCopy.querySelector(".index-td");
        let packageNameElement = templateCopy.querySelector(".package-name-td");
        let nodeNameElement = templateCopy.querySelector(".node-name-td");
        let delayElement = templateCopy.querySelector(".delay-td");

        // fill the new row with data
        indexElement.innerText = i;
        packageNameElement.innerText = autostartList[i].packageName;
        nodeNameElement.innerText = autostartList[i].nodeName;
        delayElement.innerText = autostartList[i].delay;

        autostartElements.appendChild(templateCopy);
    }

    // clear the table
    autostartTableBody.innerHTML = "";
    // put the new content into the table
    autostartTableBody.appendChild(autostartElements);
}

async function loadPresets() {
    // get JSON from API
    let response = await ApiRequests.getAutostartPresets();

    // fill info to the global variable to find preset content later
    presets = await response.json();

    let presetElements = new DocumentFragment();

    // iterate over the keys in the presets obejct, which are the names of the presets
    for (let presetName in presets) {
        let templateCopy = presetEntryTemplate.content.cloneNode(true);

        let presetNameElement = templateCopy.querySelector(".preset-name-p");
        let viewButton = templateCopy.querySelector(".view-button");
        let loadButton = templateCopy.querySelector(".load-button");
        let removeButton = templateCopy.querySelector(".remove-button");

        // display an overview
        presetNameElement.innerText = presetName;

        // register event handlers for the buttons
        addDisablerButtonHandler(loadButton, async () => {
            await ApiRequests.postAutostartPresets(presetName);
            await loadAutostart();
        });
        addDisablerButtonHandler(removeButton, async () => {
            await ApiRequests.deleteAutostartPresets(presetName);
            await loadPresets();
        });
        addDisablerButtonHandler(viewButton, () => {
            viewPresetModalHeading.innerText = presetName;
            viewPresetModalContentField.innerText = JSON.stringify(presets[presetName], null, 2);
        });

        presetElements.appendChild(templateCopy);
    }

    // clear the preset list
    presetContainer.innerHTML = "";
    // display the new content
    presetContainer.appendChild(presetElements);
}

/**
 * Requests the log messages for the currently selected node regularly and updates the HTML accordingly.
 * DO NOT await since this might loop for a long time.
 */
async function refreshLogPeriodically() {

    while (true) {
        // stop logging when the pause button is toggled to active or when the selected node changes
        let isPaused = logPauseToggleButton.classList.contains("active");
        if (!isPaused ) {
            
            await loadLog(selectedNode.packageName, selectedNode.nodeName);
        }

        // wait before next request
        await delay(LOG_LIST_REFRESH_COOLDOWN);
    }
}

function isNodeListEntryActive(nodeListEntry) {
    // TODO
    return true;
}

/**
 * Requests the log messages for the specified node and displays the content
 * @param {string} packageName The name of the package the node to log messages for is in
 * @param {string} nodeName The name of the node to log messages for
 */
async function loadLog(packageName, nodeName) {
    let response = await ApiRequests.getLog(packageName, nodeName, LOG_REQUEST_SECONDS);

    // make sure to only parse successfull requests
    if (response.status !== 200) {
        logField.innerText = "Keine Logs verfügbar";
        return;
    }

    let logs = await response.json();
    let logText = logs.join(" ");

    // put a default info text if there are no logs
    if (logText.trim() === "") {
        logText = "Keine aktuellen Log-Nachrichten";
    }

    // set the text to the log field
    logField.innerText = logText;

    // scroll text field to bottom
    logContainer.scrollTop = logContainer.scrollHeight;
}

/**
 * Start to refresh the node list regularly. 
 * DO NOT await this funtion since it loops forever and will never return.
 */
async function refreshNodeListPeriodically() {
    while (true) {
        await loadNodeList();
        // wait before next refresh
        await delay(NODE_LIST_REFRESH_COOLDOWN);
    }
}

/**
 * Requests the node list and updates the displayed list if anything changed
 */
async function loadNodeList() {
    // get list from API
    let response = await ApiRequests.getList();
    let responseBody = await response.text();

    // only rebuild the html when there actually was a change
    if (responseBody === latestNodeListResponse) {
        return;
    }
    latestNodeListResponse = responseBody;

    let nodes = JSON.parse(responseBody);

    // update node info
    nodeList = nodes;

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
        let infoColumns = templateCopy.querySelectorAll(".node-info-col");

        // text elements with data
        packageNameElement.innerText = nodeInfo.packageName;
        nodeNameElement.innerText = nodeInfo.nodeName;
        fileNameElement.innerText = nodeInfo.fileName;

        // hide the build warning icon when the node is up to date
        if (nodeInfo.isUpToDate) {
            buildIcon.toggleAttribute("hidden", true);
        }

        // hide the button that makes no sense in current state
        if (nodeInfo.isRunning) {
            startButton.toggleAttribute("hidden", true);
        } else {
            stopButton.toggleAttribute("hidden", true);
        }

        // add event handler on the whole entry except the buttons to make the node list entries collapsible
        infoColumns.forEach((col) => col.addEventListener("click", () => selectEntry(entry)));
        // entry.addEventListener("click", () => selectEntry(entry));

        // add event handlers for the buttons
        addDisablerButtonHandler(startButton, async () => {
            await ApiRequests.postStart(nodeInfo.packageName, nodeInfo.nodeName);
            await loadNodeList();
        });
        addDisablerButtonHandler(stopButton, async () => {
            await ApiRequests.postStop(nodeInfo.packageName, nodeInfo.nodeName);
            await loadNodeList();
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

    // update log immediately
    loadLog(packageName, nodeName);
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


/**
 * Can be awaited to delay the following code. This doesn't block the thread, only the async method it is called in.
 * @param {number} durationMilliseconds The duration to sleep in milliseconds
 */
async function delay(durationMilliseconds) {
    await new Promise((resolve) => setTimeout(resolve, durationMilliseconds));
}