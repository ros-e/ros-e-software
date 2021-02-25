/*
This module contains all methods that directly interact with the API
*/

import { API_URL } from "./config.js";

const statusField = document.getElementById("status-field");


export async function listGet() {
    return request("/list", "GET");
}

export async function startPost(packageName, nodeName) {
    return requestAndShowStatus("/start", "POST", {
        packageName: packageName,
        nodeName: nodeName
    });
}

export async function stopPost(packageName, nodeName) {
    return requestAndShowStatus("/stop", "POST", {
        packageName: packageName,
        nodeName: nodeName
    });
}

export async function buildPost(packageName, nodeName) {
    return requestAndShowStatus("/build", "POST", {
        packageName: packageName,
        nodeName: nodeName
    });
}

export async function buildModifiedPost() {
    let response = await request("/buildModified", "POST");
    statusField.innerText = (await response.json()).stdout.trim();
}

export async function buildAllPost() {
    let response = await request("/buildAll", "POST");
    statusField.innerText = (await response.json()).stdout.trim();
}

export async function logGet(packageName, nodeName, seconds) {
    return request("/log", "GET", {
        packageName: packageName,
        nodeName: nodeName,
        seconds: seconds
    });
}

export async function autostartGet() {
    return request("/autostart", "GET");
}

export async function autostartPut(packageName, nodeName, delayMs, index) {
    return requestAndShowStatus("/autostart", "PUT", {
        packageName: packageName,
        nodeName: nodeName,
        delayMs: delayMs,
        index: index
    });
}

export async function autostartDelete(index) {
    return requestAndShowStatus("/autostart", "DELETE", {
        index: index
    });
}

export async function autostartPresetsGet() {
    return request("/autostart/presets", "GET");
}

export async function autostartPresetsPost(presetName) {
    return requestAndShowStatus("/autostart/presets", "POST", {
        presetName: presetName 
    });
}

export async function autostartPresetsPut(presetName) {
    return requestAndShowStatus("/autostart/presets", "PUT", {
        presetName: presetName 
    });
}

export async function autostartPresetsDelete(presetName) {
    return requestAndShowStatus("/autostart/presets", "delete", {
        presetName: presetName 
    });
}

/**
 * Helper method to make fetch requests less redundant. Alerts Errors to the user or puts success message on banner.
 * @param {string} endpoint The address suffix to send the request to. Must start with a slash
 * @param {"GET"|"POST"|"PUT"|"DELETE"} method The request method
 * @param {Object} args An object of key-value pairs to send as arguments in the request
 */
async function requestAndShowStatus(endpoint, method, args = undefined) {

    let response = await request(endpoint, method, args);

    // tell status to user
    if (response.status === 200) {
        statusField.innerText = method + " to " + endpoint + " returned " + response.statusText;
    } else {
        alert(await response.text());
    }

    return response;
}


/**
 * Helper method to make fetch requests less redundant.
 * @param {string} endpoint The address suffix to send the request to. Must start with a slash
 * @param {"GET"|"POST"|"PUT"|"DELETE"} method The request method
 * @param {Object} args An object of key-value pairs to send as arguments in the request
 */
async function request(endpoint, method, args = undefined) {

    // if the bodyArgs is truthy, send it as body in json format. Otherwise don't send a body
    let bodyJson = args && method !== "GET" ? JSON.stringify(args) : undefined;

    let queryString = args && method === "GET" ? (new URLSearchParams(args)).toString() : "";

    try {
        return fetch(API_URL + endpoint + "?" + queryString, {
            method: method,
            headers: { "Content-Type": "application/json;charset=utf-8" },
            body: bodyJson
        });
    } catch (e) {
        alert(e);
    }
}
