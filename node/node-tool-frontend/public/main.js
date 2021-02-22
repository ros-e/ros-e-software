/*
Main module that handles user interaction
*/

import * as ApiRequests from "./api_requests.js";
import { NODE_LIST_REFRESH_COOLDOWN } from "./config.js";

// variables for quick access to important DOM elements
const nodeListContainer = document.getElementById("node-list-container");
/** @type {HTMLTemplateElement} */
const nodeListEntryTemplate = document.getElementById("node-list-entry-template");

// Stores the latest requested node list. Used to check for changes in the list.
let latestNodeListJson = "";

// The node currently selected in the list and whose log messages to show
const selectedNode = {
    packageName: "",
    nodeName: ""
}

// keep the node list up-to-date by refreshing it regularly
refreshNodeListPeriodically();

async function refreshNodeListPeriodically() {
    while (true) {
        refreshNodeList();
        // wait until next refresh
        await new Promise((resolve) => setTimeout(resolve, NODE_LIST_REFRESH_COOLDOWN));
    }
}

async function refreshNodeList() {
    // get list from API
    let response = await ApiRequests.getList();
    let responseBody = await response.text();

    // only rebuild the html when there actually was a change
    if (responseBody === latestNodeListJson) {
        return;
    }
    latestNodeListJson = responseBody;

    let nodes = JSON.parse(responseBody);

    // clear html list
    nodeListContainer.innerHTML = "";

    // convert each object from the node list to an HTML Element and append
    let newElement = nodeListToDocumentFragment(nodes);
    nodeListContainer.appendChild(newElement);

    // if the previously selected node is still in the list, show it in the view
    expandSelectedEntryOnly();
}

function nodeListToDocumentFragment(nodes) {

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
        let buildIcon = templateCopy.querySelector(".build-warning-icon");
        let startButton = templateCopy.querySelector(".start-button");
        let stopButton = templateCopy.querySelector(".stop-button");
        let fileNameElement = templateCopy.querySelector(".file-name-p");

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

        // add event handler to make the node list entries collapsible
        entry.addEventListener("click", () => selectEntry(entry));

        // add event handlers for the buttons
        startButton.addEventListener("click", () => ApiRequests.postStart(nodeInfo.packageName, nodeInfo.nodeName));
        stopButton.addEventListener("click", () => ApiRequests.postStop(nodeInfo.packageName, nodeInfo.nodeName));

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
            extraRow.removeAttribute("hidden");
            entry.classList.add("bg-light");
        } else {
            extraRow.setAttribute("hidden", "");
            entry.classList.remove("bg-light");
        }
    }
}

