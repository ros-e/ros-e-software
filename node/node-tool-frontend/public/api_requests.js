/*
This module contains all methods that directly interact with the API
*/

import { API_URL } from "./config.js";

export async function getList() {
    return customFetch("/list", "GET");
}

export async function postStart(packageName, nodeName) {

    let response = await customFetch("/start", "POST", {
        packageName: packageName,
        nodeName: nodeName
    });

    // alert errors to the user
    if (response.status !== 200) {
        alert(await response.text());
    }
}

export async function postStop(packageName, nodeName) {

    let response = await customFetch("/stop", "POST", {
        packageName: packageName,
        nodeName: nodeName
    });

    // alert errors to the user
    if (response.status !== 200) {
        alert(await response.text());
    }
}


/**
 * Helper method to make fetch requests less redundant
 * @param {string} endpoint The address suffix to send the request to. Must start with a slash
 * @param {"GET"|"POST"|"PUT"|"DELETE"} method The request method
 * @param {Object} bodyArgs An object of key-value pairs to send as arguments in the request body
 */
async function customFetch(endpoint, method, bodyArgs=undefined) {
    
    // if the bodyArgs is truthy, send it as body in json format. Otherwise don't send a body
    let bodyJson = bodyArgs ? JSON.stringify(bodyArgs) : undefined;

    return fetch(API_URL + endpoint, {
        method: method,
        headers: { "Content-Type": "application/json;charset=utf-8" },
        body: bodyJson
    });
}
