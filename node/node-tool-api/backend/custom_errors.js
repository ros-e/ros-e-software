/**
 * Custom errors for to distinguish between user mistakes from internal logic errors
 * 
 * Johannes Sommerfeldt, 2021-02
 */

/**
 * Used to indicate that the user specified arguments that the server can't handle
 */
module.exports.InvalidArgumentError = class InvalidArgumentError extends Error {
    constructor(message) {
        super(message);
        this.name = this.constructor.name;
    }
}

/**
 * Used to indicate that the user requests something the program is currently in the wrong state for,
 * like starting something that has already been started.
 */
module.exports.WrongStateError =  class WrongStateError extends Error {
    constructor(message) {
        super(message);
        this.name = this.constructor.name;
    }  
}

