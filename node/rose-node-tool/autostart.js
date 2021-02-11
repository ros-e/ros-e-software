/**
 * Handles everything about autostart configurations
 * 
 * Johannes Sommerfeldt, 2021-02
 */

const custom_errors = require("./custom_errors");

const fsp = require("fs/promises");

/**
 * Data access object for a sigle autostart file
 */
module.exports = class AutostartConfiguration {

    /**
     * Path to the file this object represents
     * @type {string}
     */
    #filePath;

    /**
     * Creates a access object for the configuration. 
     * Creating this does not mean a file exists for an object yet.
     * @param {string} filePath The path to the file storing this config
     */
    constructor(filePath) {
        this.#filePath = filePath;
    }

    /**
     * Returns the entries of the autostart configuration
     * @returns {Promise<AutostartEntry[]>} The entries of the autostart configuration
     */
    async getConfig() {
        let config = await this.#readConfig();
        return config;
    }

    /**
     * Overwrites the current content for this autostart configuration
     * @param {Promise<AutostartEntry[]>} config The entries of the autostart configuration to set
     */
    async setConfig(config) {
        await this.#writeConfig(config);
    }

    /**
     * Adds a node to this autostart configuration. 
     * @param {string} packageName The name of the package the node is in
     * @param {string} nodeName The name of the node to add to the autostart configuration
     * @param {number} index An integer specifying the index where to insert the node within the configuration
     * @param {number} delayMs The delay relative to the previous node to start the node with (in milliseconds)
     */
    async insert(packageName, nodeName, index, delayMs) {

        let config = await this.#readConfig();

        // if the index is not valid, set it to the last possible index (= append entry)
        if (index == null || index < 0 || index > config.length) {
            index = config.length;
        }

        // assemble the new entry
        let newEntry = new AutostartEntry(packageName, nodeName, delayMs);

        // insert the new entry at the specified index
        config.splice(index, 0, newEntry);

        // write the changes to the file
        await this.#writeConfig(config);
    }

    /**
     * Removes an entry from the autostart configuration
     * @param {number} index An integer specifying the index of the entry to delete
     */
    async remove(index) {

        let config = await this.#readConfig();

        // make sure the index is valid
        if (index == null || index < 0 || index > config.length) {
            throw new custom_errors.InvalidArgumentError("Invalid index: " + index);
        }

        // remove the value at the specified index
        config.splice(index, 1);

        //write changes to the file
        await this.#writeConfig(config);
    }

    /**
     * Returns wether this autostart configuration handler currently has a file storing data
     */
    async hasFile() {
        try {
            await fsp.access(this.#filePath);
            return true;
        } catch {
            return false;
        }
    }

    /**
     * Deletes the underlyring file for this autostart configuration
     * @throws {WrongStateError} When the file cannot be deleted, most likely due to not existing
     */
    async deleteFile() {
        try {
            await fsp.rm(this.#filePath);
        } catch (e) {
            throw new custom_errors.WrongStateError("File cannot be deleted. Reason: " + e.message);
        }
    }


    /**
     * Reads the configuration file and parses its content to a list of objects
     * @returns {Promise<AutostartEntry[]>} A list of objects containing autostart data, 
     * or an empty list if the file can't be read 
     * @throws if the file does not exists
     */
    async #readConfig() {
        try {
            let data = await fsp.readFile(this.#filePath);

            // try to parse the file data
            let config = JSON.parse(data);
            return config;

        } catch (e) {
            // console.log(e);
            // return an empty list if the file reading or parsing had errors
            return [];
        }
    }

    /**
     * Writes the autostart data to the configuration file
     * @param {AutostartEntry[]} data A list of JSON objects that will be stringified and written to the file
     */
    async #writeConfig(data) {
        let config = JSON.stringify(data, null, 2);

        await fsp.writeFile(this.#filePath, config);
    }
}

/**
 * Defines JSON objects used to store and retrieve the autostart data
 */
class AutostartEntry {
    packageName;
    nodeName;
    delay;

    constructor(packageName, nodeName, delay) {
        this.packageName = packageName;
        this.nodeName = nodeName;
        this.delay = delay;
    }
}