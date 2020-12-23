
var express = require("express");

// Import routes
var r_i2c = require('./routes/i2c');


var app = express();


// Link statics
app.use(express.static('public'));


// Link routes
app.listen(3000, () => {
 console.log("Server running on port 3000");
});


app.get("/", (req, res, next) => {
    res.json(["Tony","Lisa","Michael","Ginger","Food"]);
});


app.use('/i2c', r_i2c);

