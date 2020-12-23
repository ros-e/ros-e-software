
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


// var deltaX, deltaY;



