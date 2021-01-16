
var express = require("express");

// Import routes
var r_i2c = require('./routes/i2c');
var r_redis = require('./routes/redis');
var r_mouth = require('./routes/mouth');


var app = express();
app.listen(3001, () => {
 console.log("Server running on port 3001");
});


// Link routes
app.get("/", (req, res, next) => {
    res.json(["Tony","Lisa","Michael","Ginger","Food"]);
});


app.use('/i2c', r_i2c);
app.use('/redis', r_redis);
app.use('/mouth', r_mouth);

