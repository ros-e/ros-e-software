"use strict";
/*
Main module that handles user interaction
*/

import * as ApiRequests from "./api_requests.js";
import { LOG_REFRESH_COOLDOWN, LOG_TIMESPAN_SECONDS, LISTS_REFRESH_COOLDOWN } from "./config.js";

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
const refreshButton = document.getElementById("refresh-button");
const autoRefreshToggle = document.getElementById("auto-refresh-toggle");


/* STATIC BUTTON EVENT HANDLERS */

// refresh button
addDisablerButtonHandler(refreshButton, loadLists);

// build buttons
addDisablerButtonHandler(buildAllButton, async function () {
    await ApiRequests.buildAllPost();
    await loadNodeList();
});
addDisablerButtonHandler(buildModifiedButton, async function () {
    await ApiRequests.buildModifiedPost();
    await loadNodeList();
});

// autostart modal confirm buttons
addDisablerButtonHandler(autostartAddOkButton, async function () {
    let packageName = document.getElementById("add-autostart-package-name-input").value;
    let nodeName = document.getElementById("add-autostart-node-name-input").value;
    let delay = document.getElementById("add-autostart-delay-input").value;
    let index = document.getElementById("add-autostart-index-input").value;

    // send request
    await ApiRequests.autostartPut(packageName, nodeName, delay, index);
    // refresh displayed list
    await loadAutostart();
});
addDisablerButtonHandler(autostartRemoveOkButton, async function () {
    let index = document.getElementById("remove-autostart-index-input").value;
    await ApiRequests.autostartDelete(index);
    await loadAutostart();
});
addDisablerButtonHandler(autostartSaveOkButton, async function () {
    let presetName = document.getElementById("save-autostart-preset-name-input").value;
    await ApiRequests.autostartPresetsPut(presetName);
    await loadPresets();
});


/* global vaiables that determine the state of the data */

// Stores the latest requested node list. Used to check for changes in the list.
let latestNodeListResponse = "";

/**
 * Stores parsed node list info to check content on demand
 * @typedef {{packageName: string, nodeName: string, fileName: string, isRunning: boolean, isUpToDate: boolean}} NodeInfo
 * @type {NodeInfo[]}
 */
let nodeList = [];

// represents the currently selected node in the nodeList and node-list-container
let selectedIndex = -1;

// Stores the current preset info to check content on demand
let presets = {};


/* initialization of dynamic elements */

// init the lists
loadLists();

// keep displayed list up-to-date while the toggle is checked
refreshListsPeriodically();

// keep logs up-to-date
refreshLogPeriodically();


// =========================================================================
// FUNCTIONS


/**
 * Start to refresh the node list, autostart list and preset list regularly, unless the toggle is off
 * DO NOT await this funtion since it loops forever and will never return.
 */
async function refreshListsPeriodically() {
    while (true) {
        // wait before next refresh
        await wait(LISTS_REFRESH_COOLDOWN);

        // don't refresh if automatic refresh is disabled
        if (autoRefreshToggle.checked === false) {
            continue;
        }

        try {
            // request the info from the API and update the view
            await loadLists();

        } catch (e) {
            console.error(e);
        }
    }
}

/**
 * Requests the log messages for the currently selected node regularly and updates the HTML accordingly.
 * Skips request when no node is selected, the selected node is not running or when the pause toggle is active
 * DO NOT await since this will loop forever.
 */
async function refreshLogPeriodically() {
    while (true) {

        // wait before next request
        await wait(LOG_REFRESH_COOLDOWN);

        // don't log when no node was selected yet
        if (selectedIndex === -1) {
            continue;
        }

        // skip logging when the pause button is toggled to active or when the selected node is not running
        let isPaused = logPauseToggleButton.classList.contains("active");
        let selectedNode = nodeList[selectedIndex];

        if (isPaused || selectedNode.isRunning === false) {
            continue;
        }

        try {
            // request log data for node and update view
            await loadLog(selectedNode.packageName, selectedNode.nodeName);

        } catch (e) {
            console.error(e);
        }
    }
}

/**
 * Refreshes the node list, autostart list and preset list
 * @returns {Promise<void, void, void>}
 */
async function loadLists() {
    let nodeListPromise = loadNodeList();
    let autostartPromise = loadAutostart();
    let presetPromise = loadPresets();
    return Promise.all([nodeListPromise, autostartPromise, presetPromise]);
}

/**
 * Requests the node list and updates the displayed list if anything changed
 */
async function loadNodeList() {
    // get list from API
    let response = await ApiRequests.listGet();

    // make sure to only parse successfull requests
    if (response.status >= 400) {
        console.error("Node list request returned status: " + response.status + " "  + response.statusText);
        console.log("Skipped loading node list");
        return;
    }

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

    // since the list changed, reset the selection
    selectedIndex = -1;
}

/**
 * Requests the autostart configuration from the API and displays the content
 */
async function loadAutostart() {
    // get JSON from API
    let response = await ApiRequests.autostartGet();
    
    // make sure to only parse successfull requests
    if (response.status >= 400) {
        console.error("Autostart request returned status: " + response.status + " " + response.statusText);
        console.log("Skipped loading autostart config");
        return;
    }

    let autostartList = await response.json();

    let autostartElements = new DocumentFragment();

    for (let index = 0; index < autostartList.length; index += 1) {

        let packageName = autostartList[index].packageName;
        let nodeName = autostartList[index].nodeName;
        let delay = autostartList[index].delay;

        // create a new row for the table using the template
        let templateCopy = autostartEntryTemplate.content.cloneNode(true);

        let indexElement = templateCopy.querySelector(".index-td");
        let packageNameElement = templateCopy.querySelector(".package-name-td");
        let nodeNameElement = templateCopy.querySelector(".node-name-td");
        let delayElement = templateCopy.querySelector(".delay-td");
        let moveUpButton = templateCopy.querySelector(".move-up-button");
        let moveDownButton = templateCopy.querySelector(".move-down-button");

        // fill the new row with data
        indexElement.innerText = index;
        packageNameElement.innerText = packageName;
        nodeNameElement.innerText = nodeName;
        delayElement.innerText = delay;

        // add button handlers
        addDisablerButtonHandler(moveUpButton, async function () {
            if (index === 0) {
                // can't move the first element up any further
                return;
            }
            // also disable the other move button
            moveDownButton.toggleAttribute("disabled", true);
            // delete at old index and insert at the new one
            await ApiRequests.autostartDelete(index);
            await ApiRequests.autostartPut(packageName, nodeName, delay, index - 1);
            // update view, then allow buttons again
            await loadAutostart();
            moveDownButton.toggleAttribute("disabled", false);
        });
        addDisablerButtonHandler(moveDownButton, async function () {
            if (index === autostartList.length - 1) {
                // can't move the last element down any further
                return;
            }
            // also disable the other move button
            moveUpButton.toggleAttribute("disabled", true);
            // delete at old index and insert at the new one
            await ApiRequests.autostartDelete(index);
            await ApiRequests.autostartPut(packageName, nodeName, delay, index + 1);
            // update view, then allow buttons again
            await loadAutostart();
            moveUpButton.toggleAttribute("disabled", false);
        });

        autostartElements.appendChild(templateCopy);
    }

    // clear the table
    autostartTableBody.innerHTML = "";
    // put the new content into the table
    autostartTableBody.appendChild(autostartElements);
}

/**
 * Requests the autostart presets from the API and displays the content
 */
async function loadPresets() {
    // get JSON from API
    let response = await ApiRequests.autostartPresetsGet();
    
    // make sure to only parse successfull requests
    if (response.status >= 400) {
        console.error("Preset request returned status: " + response.status + " " + response.statusText);
        console.log("Skipped loading autostart presets");
        return;
    }

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
            await ApiRequests.autostartPresetsPost(presetName);
            await loadAutostart();
        });
        addDisablerButtonHandler(removeButton, async () => {
            await ApiRequests.autostartPresetsDelete(presetName);
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
 * Requests the log messages for the specified node and displays the content
 * @param {string} packageName The name of the package the node to log messages for is in
 * @param {string} nodeName The name of the node to log messages for
 */
async function loadLog(packageName, nodeName) {
    let response = await ApiRequests.logGet(packageName, nodeName, LOG_TIMESPAN_SECONDS);

    // make sure to only parse successfull requests
    if (response.status >= 400) {
        logField.innerText = "Keine Logs verfügbar";
        console.error("Log request returned status: " + response.status + " " + response.statusText);
        console.log("Skipped loading log");
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
 * Builds a list of HTML elements out of a list of nodes
 * @param {NodeInfo[]} nodes The list of node information as provided by the api
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

        // assign the element with an id that matches the index of the info object 
        // in the node list to retrieve info later
        entry.id = "node-list-entry-" + i;

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
            await ApiRequests.startPost(nodeInfo.packageName, nodeInfo.nodeName);
            await loadNodeList();
        });
        addDisablerButtonHandler(stopButton, async () => {
            await ApiRequests.stopPost(nodeInfo.packageName, nodeInfo.nodeName);
            await loadNodeList();
        });
        addDisablerButtonHandler(buildButton, async () => {
            await ApiRequests.buildPost(nodeInfo.packageName, nodeInfo.nodeName);
            await loadNodeList()
        });

        nodeListEntryElements.appendChild(templateCopy);
    }
    return nodeListEntryElements;
}

/**
 * Sets the clicked entry of the node list as selected
 * @param {Element} nodeListEntry The node list entry that was clicked
 */
function selectEntry(nodeListEntry) {

    // find the ID of this entry
    let nodeIndex = getIndexForNodeListEntry(nodeListEntry);

    // ignore clicks on elements that are already selected
    if (nodeIndex === selectedIndex) {
        return;
    }

    // update the variable tracking which is selected
    selectedIndex = nodeIndex;

    // update the view
    expandOnly(nodeListEntry);

    // update log immediately
    let nodeInfo = nodeList[nodeIndex];
    loadLog(nodeInfo.packageName, nodeInfo.nodeName);
}

/**
 * Gets the index of the node a node list entry represents
 * @param {Element} nodeListEntry The node-list-entry element to get the list index for
 * @returns {number} index of the node
 */
function getIndexForNodeListEntry(nodeListEntry) {
    let idString = nodeListEntry.id.replace("node-list-entry-", "");
    return parseInt(idString, 10);
}

/**
 * Collapses all elements in the node list except the selected one, which will be expanded
 * @param {Element} nodeListEntry
 */
function expandOnly(nodeListEntry) {
    for (let entry of nodeListContainer.children) {

        let extraRow = entry.querySelector(".node-list-entry-extra");

        // reveal extra row for the specified node list entry and hide it for all other node list entries
        if (entry === nodeListEntry) {
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
 * @param {number} durationMilliseconds The duration to wait in milliseconds
 * @returns {Promise<void>} A Promise that resolves after the specified time
 */
async function wait(durationMilliseconds) {
    await new Promise((resolve) => setTimeout(resolve, durationMilliseconds));
}