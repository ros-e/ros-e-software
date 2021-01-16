
var express = require("express");

// Import routes

var app = express();

// Link statics
// app.use(express.static('public'));
app.use("/view", express.static(__dirname + '/public'));


app.listen(3000, () => {
    console.log("Server running on port 3000");
   });



// Link routes
app.get("/", (req, res, next) => {
    res.json(["Frontend site"]);
});

// Link routes
app.get("/system", (req, res, next) => {
    res.json(["System"]);
});

