

// let svg_mouth = document.getElementById("svg_mouth");

let svg_mouth = d3.select("#svg_mouth");

class LED {

    constructor(idx, row, col, r = 0, g = 0, b = 0) {
        this.idx = idx;
        this.col = col;
        this.row = row;
        this.active = false;
        this.r = 0;
        this.g = 0;
        this.b = 0;
    }
}

var leds = Array();
var ledCount = 16;
var ledsPerRow = 8;

for (let i = 0; i < ledCount; i++) {
    let index = i;
    if (i <= 7) index = 7 - i;

    let row = Math.floor(i / ledsPerRow);
    let col = i % ledsPerRow;

    leds.push(new LED(index, row, col));
}







// Redraw based on the new size whenever the browser window is resized.
window.addEventListener("resize", updateSize);
window.addEventListener("load", updateSize);

let percMargin = 0.01;
let percMarginLeftRight = 0.1;


function updateSize() {
    let svgSize = svg_mouth.node().getBoundingClientRect();

    let width = Math.floor(svgSize.width);

    let m = Math.floor(width * percMargin);
    let mlr = Math.floor(width * percMarginLeftRight);

    let restWidth = width - 2 * mlr - (ledsPerRow - 1) * m;
    let rectSize = Math.floor(restWidth / ledsPerRow);

    svg_mouth.selectAll("rect").data(leds)
        .attr("width", rectSize)
        .attr("height", rectSize)
        .attr("x", d => mlr + d.col * (rectSize + m))
        .attr("y", d => d.row * (rectSize + m))
        .attr("stroke", "black")
        .attr("stroke-width", "2px")
        .attr("rx", "2px")
        .attr("ry", "2px")
        .attr("stroke-opacity", 0.5)
        .on("click", (e, d) => {
            d.active = !d.active;
            uptadeLEDs();
        })
        ;

    let rows = Math.ceil(ledCount / ledsPerRow);

    svg_mouth.attr("height", rows * rectSize + (rows - 1) * m)
}


svg_mouth.on("resize", function () {
    console.log("Resize SVG");
})


svg_mouth.selectAll("rect").data(leds).enter().append("rect")
    .attr("index", d => d.idx)
    .attr("row", function (d) { return d.row })
    .attr("col", function (d) { return d.col })
    ;



function uptadeLEDs() {
    svg_mouth.selectAll("rect").data(leds)
        .attr("fill", d => d.active ? "white" : "black")
        ;

    let addr = 0x20;
    let cmd = 0x10;

    let data = [];
    data.push(0);
    data.push(0);
    data.push(0x10);
    data.push(0x10);
    data.push(0x10);

    
    leds.forEach(led => {
        if (led.active) {
            let i = led.idx
            if (i <= 7) data[1] |= (0x01 << i);
            else data[0] |= (0x01 << (i - 8));
        }
    });


    let js = {
        addr: addr,
        cmd: cmd,
        data: data
    }

    let url = new URL("/i2c/transmit", window.location.origin);

    console.log("Fetch");
    console.log(js);

    fetch(url, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        },
        // body: '{"test":"test"}'
        body: JSON.stringify(js)
    })    
    .then(res => res.json())
    .then(function(res) {
        console.log('Data was posted');
        console.log(res)
    })
    .catch(function(error) {
      console.log(error);
    });



}





