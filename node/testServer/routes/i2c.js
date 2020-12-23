var express = require('express');

const r = require("../utils/redis");

var router = express.Router();

// middleware that is specific to this router
router.use(function timeLog(req, res, next) {
  console.log('Time: ', Date.now());
  next();
});


// define the home page route
router.get('/transmit', function(req, res) {

    let addr = parseInt(req.query.addr);
    let cmd = parseInt(req.query.cmd);
    let data = parseInt(req.query.data);

    i2cMsg = {
      topic: "system/i2c/write8",
      type: "I2Cwrite8",
      address: {
        type: "uint8",
        value: addr,
      },
      command: {
        type: "uint8",
        value: cmd,
      },
      data: {
        type: "uint8",
        value: data,
      }
    }

    r.publishToROS("message/ros/generic", i2cMsg);

    // res.send(`I2C Transmit cmd: ${cmd} and data: ${data} to addr: ${addr}`);
    res.json({response: `I2C Transmit cmd: ${cmd}(0x${cmd.toString(16)}) and data: ${data}(0x${data.toString(16)}) to addr: ${addr}(0x${addr.toString(16)})`});
});


// // define the about route
// router.get('/about', function(req, res) {
//   res.send('About birds');
// });

module.exports = router;