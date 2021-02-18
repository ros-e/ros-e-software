/*
Frontend server for the node tool API

Johannes Sommerfeldt, 2021/02
*/

const express = require("express");
const cors = require("cors");

let app = express();

app.use(cors());

// make the specified directory accessible directly in the browser
app.use(express.static(__dirname + "/public"));

// host the server
let server = app.listen(42751, () => {
    console.log("Server running on port " + server.address().port);
});
