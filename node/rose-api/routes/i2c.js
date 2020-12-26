var express = require('express');

var bodyParser = require("body-parser");

// create application/json parser
var jsonParser = bodyParser.json()
// create application/x-www-form-urlencoded parser
var urlencodedParser = bodyParser.urlencoded({ extended: false })


const r = require("../utils/redis");

var router = express.Router();

// middleware that is specific to this router
router.use(function timeLog(req, res, next) {
    console.log('[I2C Router] Time: ', Date.now());
    next();
});


function getByteArray(dataString) {

    let data = [];
    let splits = dataString.split(/[\s,;]+/)

    // console.log(splits);

    splits.forEach(s => {
        if (s.length > 0) {
            try {
                data.push(parseInt(s));
            } catch (error) {
                console.log(error);
            }
        }
    });

    return data;
}

router.get('/', function (req, res) {
    res.send("Hello I2C API");
});

// // define the home page route
// router.get('/transmit', function (req, res) {

//     let addr = parseInt(req.query.addr);
//     let cmd = parseInt(req.query.cmd);
//     let data = getByteArray(req.query.data);

//     // let i2cMsgData = [];
//     // data.forEach(d => {
//     //     i2cMsgData.push({type: "uint8", value: d});
//     // });

//     i2cMsg = {
//         topic: "system/i2c/writeArray",
//         type: "I2CwriteArray",
//         address: {
//             type: "uint8",
//             value: addr,
//         },
//         command: {
//             type: "uint8",
//             value: cmd,
//         },
//         data: {
//             type: "uint8[]",
//             value: data,
//         }
//     }

//     r.publishToROS("message/ros/generic", i2cMsg);

//     // res.send(`I2C Transmit cmd: ${cmd} and data: ${data} to addr: ${addr}`);
//     res.json({
//         redisRosMsg: i2cMsg,
//         response: `I2C Transmit cmd: ${cmd}(0x${cmd.toString(16)}) and data: ${data}(0x${data.toString(16)}) to addr: ${addr}(0x${addr.toString(16)})`
//     });
// });

router.get('/transmit', function (req, res) {
    res.send("Use POST to post some data");
});

// define the home page route
router.post('/transmit', jsonParser, function (req, res) {
    let js = req.body

    let resJson = {errors: []}

    let addr = js.addr;
    let cmd = js.cmd;
    let data = js.data == undefined ? [] : js.data;

    if (addr == undefined) resJson.errors.push("I2C Address ist not defined");
    if (cmd == undefined) resJson.errors.push("I2C Cmd ist not defined");

    if (resJson.errors.length == 0) {
        i2cMsg = {
            topic: "system/i2c/writeArray",
            type: "I2CwriteArray",
            address: {
                type: "uint8",
                value: addr,
            },
            command: {
                type: "uint8",
                value: cmd,
            },
            data: {
                type: "uint8[]",
                value: data,
            }
        }
    
        r.publishToROS("message/ros/generic", i2cMsg);

        resJson.redisRosMsg = i2cMsg;
        resJson.response = `I2C Transmit cmd: ${cmd}(0x${cmd.toString(16)}) and data: ${data}(0x${data.toString(16)}) to addr: ${addr}(0x${addr.toString(16)})`;
        
    }

    res.json(resJson);
});


// // define the about route
// router.get('/about', function(req, res) {
//   res.send('About birds');
// });

module.exports = router;