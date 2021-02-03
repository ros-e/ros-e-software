/**
 * Handles everything about autostart configurations
 * 
 * Johannes Sommerfeldt, 2021-02
 */

const fs = require("fs");

module.exports = class AutostartConfiguration {

    #filePath;

    constructor(filePath) {
        this.#filePath = filePath;
    }

    async list() {
        return await this.#readConfig();
    }

    async insert(packageName, nodeName, index, delayMs) {

        let data = await this.#readConfig();
        let newEntry = {
            packageName: packageName,
            nodeName: nodeName,
            delay: delayMs
        }
        // TODO insert at index instead
        data.push(newEntry);

        this.#writeConfig(data);
    }

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

                } catch {
                    // or return an empty list if the file or the parsing had errors
                    resolve([]);
                } 
            });
        });
    }

    async #writeConfig(data) {
        let config = JSON.stringify(data, null, 2);
        fs.writeFile(this.#filePath, config, (err) => {
            if (err) {
                throw err;
            }
        });
    }
}