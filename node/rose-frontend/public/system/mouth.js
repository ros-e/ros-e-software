

// let svg_mouth = document.getElementById("svg_mouth");

// let urlBase = window.location.origin

let svg_mouth = d3.select("#svg_mouth");

let range_mouthR = document.getElementById("range_mouthR");
let range_mouthG = document.getElementById("range_mouthG");
let range_mouthB = document.getElementById("range_mouthB");

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



// Color pickers
range_mouthR.addEventListener("input", updateColor)
range_mouthG.addEventListener("input", updateColor)
range_mouthB.addEventListener("input", updateColor)

let do_updateLEDs = false;
let interval_updateLEDs = setInterval(uptadeLEDs, 100);

function updateColor() {

    let r = Math.floor(range_mouthR.value);
    let g = Math.floor(range_mouthG.value);
    let b = Math.floor(range_mouthB.value);

    // console.log(`${r} ${g} ${b}`)

    leds.forEach(l => {
        if (l.active) {
            l.r = r;
            l.g = g;
            l.b = b;
        }
    });

    updateVisuals();

    do_updateLEDs = true;
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



    svg_mouth.selectAll(".mouthLEDs").data(leds)
        .attr("width", rectSize)
        .attr("height", rectSize)
        .attr("x", d => mlr + d.col * (rectSize + m))
        .attr("y", d => d.row * (rectSize + m))
        .attr("stroke", "white")
        .attr("stroke-width", `${Math.max(Math.floor(m/4) * 2 + 1, 1)}px`)
        .attr("rx", "2px")
        .attr("ry", "2px")
        .on("click", (e, d) => {
            console.log("Click rect");
            d.active = !d.active;
            e.stopPropagation();
            updateVisuals();
            // uptadeLEDs();
        })
        ;

        updateVisuals();

    let rows = Math.ceil(ledCount / ledsPerRow);

    svg_mouth.attr("height", rows * rectSize + (rows - 1) * m)
}


svg_mouth.on("resize", function () {
    console.log("Resize SVG");
})


svg_mouth.append("rect")
    .classed("mouthBG", true)
    .attr("width", "100%")
    .attr("height", "100%")
    .attr("fill", "gray")

svg_mouth.selectAll(".mouthLEDs").data(leds).enter().append("rect")
    .classed("mouthLEDs", true)
    .attr("index", d => d.idx)
    .attr("row", function (d) { return d.row })
    .attr("col", function (d) { return d.col })
    ;

svg_mouth.on("click", (e) => {
    console.log("Click clack");
    leds.forEach(l => {
        l.active = false;
    });
    updateVisuals();
})


function updateVisuals() {
    svg_mouth.selectAll(".mouthLEDs").data(leds)
        // .attr("fill", d => d.active ? "white" : "black")
        .attr("stroke-opacity", (d) => (d.active ? 1 : 0.0) )
        .attr("fill", (d) => `rgb(${d.r},${d.g},${d.b})`)
        ;
}

function uptadeLEDs() {

    if (!do_updateLEDs) return;

    do_updateLEDs = false;

    let js = {leds: []}

    leds.forEach(led => {
        js.leds.push({
            id: led.idx,
            r: led.r,
            g: led.g,
            b: led.b
        })
    });
    
    
    let url = new URL("api/mouth", urlBase);
    

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

    return;
}





