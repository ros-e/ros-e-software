/**
 * REST-API for the Node-Tool
 * 
 * Johannes Sommerfeldt, 2021-02
 */

const NodeTool = require("./node_tool");
const custom_errors = require("./custom_errors");
const config = require("./config");

const express = require("express");
const bodyParser = require("body-parser");


// ==================================================================
// PREPARE BACKEND

// The object handling all backend logic
let nodeTool = new NodeTool();

// register event handlers to shut down all nodes when this program is terminated
["SIGINT", "SIGTERM", /*"uncaughtException"*/].forEach((event) => {
    process.on(event, async () => {
        await nodeTool.stopAll();
        console.log(event + "! Stopped all nodes");
        process.exit();
    });
});


// ==================================================================
// PREPARE API

// create express application
let app = express();

// set request data parsers
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: true }));

// set API endpoints
app.get("/", (req, res, next) => {
    res.send("Hello, this is the test home for the Node-Tool API for ROS-E")
    // TODO send frontend page here?
});

app.get("/list", async (req, res, next) => {
    let backendMethod = nodeTool.listNodes.bind(nodeTool);
    await handleRequest(req, res, next, backendMethod);
});

app.post("/start", async (req, res, next) => {
    let backendMethod = nodeTool.startNode.bind(nodeTool);
    let requiredParams = ["packageName", "nodeName"];
    await handleRequest(req, res, next, backendMethod, requiredParams);
});

app.post("/stop", async (req, res, next) => {
    let backendMethod = nodeTool.stopNode.bind(nodeTool);
    let requiredParams = ["packageName", "nodeName"];
    await handleRequest(req, res, next, backendMethod, requiredParams);
});

app.get("/log", async (req, res, next) => {
    let backendMethod = nodeTool.getLogs.bind(nodeTool);
    let requiredParams = ["packageName", "nodeName", "seconds"];
    await handleRequest(req, res, next, backendMethod, requiredParams);
});

app.get("/autostart", async (req, res, next) => {
    let backendMethod = nodeTool.getAutostart.bind(nodeTool);
    await handleRequest(req, res, next, backendMethod);
});

app.put("/autostart", async (req, res, next) => {
    let backendMethod = nodeTool.addAutostart.bind(nodeTool);
    let requiredParams = ["packageName", "nodeName", "index", "delayMs"];
    await handleRequest(req, res, next, backendMethod, requiredParams);
});

app.delete("/autostart", async (req, res, next) => {
    let backendMethod = nodeTool.removeAutostart.bind(nodeTool);
    let requiredParams = ["index"];
    await handleRequest(req, res, next, backendMethod, requiredParams);
});

app.get("/autostart/presets", async (req, res, next) => {
    let backendMethod = nodeTool.listAutostartPresets.bind(nodeTool);
    await handleRequest(req, res, next, backendMethod);
});

app.post("/autostart/presets", async (req, res, next) => {
    let backendMethod = nodeTool.loadAutostartPreset.bind(nodeTool);
    let requiredParams = ["presetName"];
    await handleRequest(req, res, next, backendMethod, requiredParams);
});

app.put("/autostart/presets", async (req, res, next) => {
    let backendMethod = nodeTool.saveAsAutostartPreset.bind(nodeTool);
    let requiredParams = ["presetName"];
    await handleRequest(req, res, next, backendMethod, requiredParams);
});

app.delete("/autostart/presets", async (req, res, next) => {
    let backendMethod = nodeTool.deleteAutostartPreset.bind(nodeTool);
    let requiredParams = ["presetName"];
    await handleRequest(req, res, next, backendMethod, requiredParams);
});

app.post("/build", async (req, res, next) => {
    let backendMethod = nodeTool.buildPackages.bind(nodeTool);
    let requiredParams = ["packageName"];
    await handleRequest(req, res, next, backendMethod, requiredParams);
});

app.post("/buildAll", async (req, res, next) => {
    let backendMethod = nodeTool.buildPackages.bind(nodeTool);
    await handleRequest(req, res, next, backendMethod);
});


// set an error handler middleware to be called whenever there is an Error in any endpoint function
app.use(errorHandler);


// ==============================================================================
// start the service

// immediate invoked function exrpression requires to run async code in-order within synchronous context
(async function () {

    // run the autostart 
    console.log("Initiating autostart...");
    await nodeTool.runAutostart();
    console.log("Autostart done.");

    // start the server (port was chosen at will, feel free to change)
    let server = app.listen(config.SERVER_PORT, () => {
        let address = server.address().address;
        let port = server.address().port;
        console.log("Server now listens at: " + address + ":" + port);
    });
})();



// =======================================================================
// handler and helper fucntions

/**
 * Runs the specified function with the parsed values for the specified parameters.
 * This method fully covers error handling and responding to the client.
 * @param {Request} req The request object of this API endpoint
 * @param {Response} res The response object of this API endpoint
 * @param {function} next the next() function from the express logic
 * @param {function} fn The function with the backend logic whose return value will be sent to the client
 * @param {string[]} requiredParams The names of the parameters that must be in the request,
 * the parsed values of which will be passed to the specified function
 */
async function handleRequest(req, res, next, fn, requiredParams = []) {
    try {
        // make sure the required request parameters are given in the request and parse them
        let parsedParams = parseParams(req, requiredParams);

        // call the function that handles the backend logic for the request
        let returnVal = await fn(...parsedParams);

        // send response, or just "OK" when the function does not return a value
        if (returnVal !== undefined) {
            res.status(200).end(returnVal);
        } else {
            res.sendStatus(200);
        }

    } catch (err) {
        // send any error that occurs to the error handler middleware
        next(err);
    }
}

/**
 * Gets the specified parameters from the parser, checks if they are defined and returns them as object
 * @param {Request} req The request object of an API endpoint method
 * @param  {string[]} paramNames The names of the parameters to parse
 * @throws {InvalidArgumentError} When any of the parameters' value is undefined
 * @return {any[]} The values parsed from the reqest
 */
function parseParams(req, paramNames) {
    let parsedParams = [];

    for (let paramName of paramNames) {
        // search for the parameter in the query string and request body
        let parsedValue = req.query[paramName];
        if (typeof parsedValue === "undefined") {
            parsedValue = req.body[paramName];
        }

        if (typeof parsedValue === "undefined") {
            throw new custom_errors.InvalidArgumentError("You must specify " + paramName);
        }

        parsedParams.push(parsedValue);
    }
    return parsedParams;
}

function errorHandler(err, req, res, next) {
    if (err instanceof custom_errors.InvalidArgumentError) {
        // The arguments make no sense, so send Error 400 Bad Request
        res.status(400).end(err.message);

    } else if (err instanceof custom_errors.WrongStateError) {
        // The current state of the node tool doesn't allow what the client tries to do,
        // so send Error 409 Conflict
        res.status(409).end(err.message);

    } else {
        // Any other error is most likely internal server logic, 
        // so log it and send Error 500 Internal Server Error
        console.error("INTERNAL ERROR:");
        console.error(err);
        res.status(500).end(err.message);
    }
}
