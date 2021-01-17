
let scale = 5;
let svgMargin = 10;

var visualizerLeft = new PolygonVisualizer("polygonVisualizerLeft", 128, 64, scale, 3, svgMargin);
var polygonColLeft = document.getElementById("polygonColLeft");

polygonColLeft.appendChild(visualizerLeft.getVisualizerDiv());
visualizerLeft.init();

var visualizerRight = new PolygonVisualizer("polygonVisualizerRight", 128, 64, scale, 3, svgMargin);
var polygonColRight = document.getElementById("polygonColRight");

polygonColRight.appendChild(visualizerRight.getVisualizerDiv());
visualizerRight.init();

visualizerLeft.synchronize(visualizerRight);




let fill = true;

let btnFillPolygons = document.getElementById("btnFillPolygons");
btnFillPolygons.addEventListener("click", function() {
    fill = !fill;
    if (visualizerLeft) visualizerLeft.setFill(fill);
    if (visualizerRight) visualizerRight.setFill(fill);

    if (fill) {
        btnFillPolygons.classList.add("btn-primary");
        btnFillPolygons.classList.remove("btn-secondary");
    } else {        
        btnFillPolygons.classList.add("btn-secondary");
        btnFillPolygons.classList.remove("btn-primary");
    }

});



let publishI2C = false;
function i2cCallback() {
    // console.log("I2C callback");
    publishI2C = true;
}


setInterval(i2cPublishAll, 100);

function toUint8(x) {
    return (Math.floor(x + 0.5) + 256) % 256;
}

function arraysEqual(a, b) {
    if (a === b) return true;
    if (a == null || b == null) return false;
    if (a.length !== b.length) return false;
  
    // If you don't care about the order of the elements inside
    // the array, you should sort both arrays here.
    // Please note that calling sort on an array will modify that array.
    // you might want to clone your array first.
  
    for (var i = 0; i < a.length; ++i) {
      if (a[i] !== b[i]) return false;
    }
    return true;
}

let lastEyeData = [];
let lastIrisData = [];
let lastViewDirectionData = [];
function i2cPublishAll() {
    if (publishI2C) {

        // EYE
        let cmd = 0x30;
        let addrLeft = 0x0A;
        let addrRight = 0x0B;

        let data = [50];

        let poly = visualizerRight.polygonDict["eye"];
        // console.log(poly.target);

        for (let i = 0; i < poly.pointCount; i++) {
            const p = poly.target[i];
            data.push(toUint8(p.x));
            data.push(toUint8(p.y));
            data.push(toUint8(p.c));
            data.push(toUint8(p.cx));
            data.push(toUint8(p.cy));
        }

        let timeout = 0;
        if (!arraysEqual(data, lastEyeData)) {
            lastEyeData = data;
            ic2Publish(addrLeft, cmd, data);
            ic2Publish(addrRight, cmd, data);
            timeout = 10;
        }

        // IRIS
        cmd = 0x28;
        data = [];

        poly = visualizerRight.polygonDict["iris"];
        // console.log(poly.target);

        for (let i = 0; i < poly.pointCount; i++) {
            const p = poly.target[i];
            data.push(toUint8(p.x));
            data.push(toUint8(p.y));
            data.push(toUint8(p.c));
            data.push(toUint8(p.cx));
            data.push(toUint8(p.cy));
        }

        if (!arraysEqual(data, lastIrisData)) {
            lastIrisData = data;
            ic2Publish(addrLeft, cmd, data);
            ic2Publish(addrRight, cmd, data);
            timeout = 10;
        }

        // View Directions
        cmd = 0xA0;
        let x = toUint8(visualizerRight.viewDirection.x);
        let y = toUint8(visualizerRight.viewDirection.y);
        data = [x, y]; 

        if (!arraysEqual(data != lastViewDirectionData)) {
            lastViewDirectionData = data;
            setTimeout(() => {
                ic2Publish(addrLeft, cmd, data);
                ic2Publish(addrRight, cmd, data);
            }, timeout);    
        }

        publishI2C = false;
    }
}

function ic2Publish(addr, cmd, data) {
    let url = new URL("api/i2c/transmit", urlBase);
    
    fetch(url, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        },
        // body: '{"test":"test"}'
        body: JSON.stringify({cmd: cmd, addr: addr, data: data})
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

visualizerRight.callbackOnPolygonUpdate = i2cCallback; 

// var deltaX, deltaY;



