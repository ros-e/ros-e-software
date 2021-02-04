/**
 * Handles everything about autostart configurations
 * 
 * Johannes Sommerfeldt, 2021-02
 */

const fs = require("fs");

/**
 * Data access object for a sigle autostart file
 */
module.exports = class AutostartConfiguration {

    #filePath;

    constructor(filePath) {
        this.#filePath = filePath;
    }

    /**
     * Returns the entries of the autostart configuration
     * @returns {Promise<AutostartEntry[]>} The entries of the autostart configuration
     */
    async list() {
        let config = await this.#readConfig();
        return config;
    }

    /**
     * Adds a node to this autostart configuration
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

        try {
            config.splice(index, 1);
        } catch {
            throw new Error("Invalid index!");
        }

        //write changes to the file
        await this.#writeConfig(config);
    }

    /**
     * Reads the configuration file and parses its content to a list of objects
     * @returns {Promise<AutostartEntry[]>} A list of objects containing autostart data, 
     * or an empty list if the file can't be read 
     */
    async #readConfig() {
        // promisify the readFile function to make the method async and non-blocking
        return new Promise((resolve, reject) => {
            fs.readFile(this.#filePath, null, (err, data) => {
                try {
                    // if the file doesn't exist, return an empty list
                    if (err) {
                        throw err;
                    }
                    // try to parse the file data
                    let config = JSON.parse(data);
                    resolve(config);

                } catch (e) {
                    console.log(e);
                    console.log("for data: " + data);
                    // or return an empty list if the file or the parsing had errors
                    resolve([]);
                }
            });
        });
    }

    /**
     * Writes the autostart data to the configuration file
     * @param {AutostartEntry[]} data A list of JSON objects that will be stringified and written to the file
     */
    async #writeConfig(data) {
        let config = JSON.stringify(data, null, 2);

        return new Promise((resolve, reject) => {
            fs.writeFile(this.#filePath, config, (err) => {
                if (err) {
                    console.log(err);
                    reject(err);
                } else {
                    resolve();
                }
            });
        });
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