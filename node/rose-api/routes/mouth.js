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
    console.log('[Redis Router] Time: ', Date.now());
    next();
});


router.get('/', function (req, res) {

    //let key = req.query.key;

    let redisKey = "mouth/leds"

    let resJson = {errors: [], leds: [] };

    r.client.get(redisKey, (err, reply) => {
        if (err != null) resJson.errors.push(err);
        resJson.leds = JSON.parse(reply);
        res.json(resJson);
    });


});

router.post('/', jsonParser, function (req, res) {

    let leds = req.body.leds;
    
    dat = [];
    leds.forEach(l => {
        dat.push({
            id: {
                type: "uint16",
                value: l.id
            },
            r: {
                type: "uint8",
                value: l.r
            },
            g:  {
                type: "uint8",
                value: l.g
            },
            b:  {
                type: "uint8",
                value: l.b
            }
        })
    });


    mouthMsg = {
        topic: "mouth/leds/updateToColor",
        type: "LEDs",
        leds: {
            type: "LED[]",
            value: dat
        }
    }

    r.publishToROS("message/ros/generic", mouthMsg);


    let resJson = {errors: []};

    resJson.redisRosMsg = mouthMsg;
    resJson.response = `Mouth Message transmitted.`;
    
    res.json(resJson);
});

module.exports = router;