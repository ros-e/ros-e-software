


class PolygonVisualizer {

    constructor(name = "", rows = 128, cols = 64, scale = 1, controlSizeFactor = 3, svgMargin = 10) {

        this.name = name;

        this.scale = scale;
        this.cols = cols;
        this.rows = rows;
        
        this.svgMargin = svgMargin;

        //this.controlSize = scale * controlSizeFactor;
        this.controlSize = controlSizeFactor;
        this.trimX = 1;
        this.trimY = 1;
        
        this.iActivePoly = 0;
        this.polygons = []
        this.polygonDict = {}

        this.setScale(this.scale);

        this.div = null;
        this.canvas = null;
        this.ctx = null;
        this.svg = null;

        this.animationDuration = 500;
        this.animationStep = 20;

        this.fpsCounter = 0;

        this.viewDirection = new ControlPoint({x: 0, y: 0});

        this.synchronizedVisualizer = [];

        this.__showFillSteps = false;

        this.__showViewLines = false;
        this.viewLines = [];
        for (let i = 0; i < 4; i++) { this.viewLines.push(new ControlPoint({}));}

        this.pressedCmd = false;
        this.snapPointsX = [];
        this.snapPointsY = [];
        
        var points = [];
        points.push(new ControlPoint({x: 15, y: 30, viewInfluence: 50}));
        points.push(new ControlPoint({x: 48, y: 30, viewInfluence: 50}));
        points.push(new ControlPoint({x: 48, y: 97, viewInfluence: 50}));
        points.push(new ControlPoint({x: 15, y: 97, viewInfluence: 50}));

        // points.push(new Point({x: 15, y: 30, cx: 30, cy: 20, viewInfluence: 50}));
        // points.push(new Point({x: 48, y: 30, cx: 57, cy: 63, viewInfluence: 50}));
        // points.push(new Point({x: 48, y: 97, cx: 31, cy: 107, viewInfluence: 50}));
        // points.push(new Point({x: 15, y: 97, cx: 6, cy: 63, viewInfluence: 50}));

        var points1 = [];
        // points1.push(new Point({x: 25, y: 45}))
        // points1.push(new Point({x: 38, y: 45}))
        // points1.push(new Point({x: 38, y: 82}))
        // points1.push(new Point({x: 25, y: 82}))

        points1.push(new ControlPoint({x: 25, y: 45, cx: 31, cy: 40}))
        points1.push(new ControlPoint({x: 38, y: 45, cx: 44, cy: 63}))
        points1.push(new ControlPoint({x: 38, y: 82, cx: 31, cy: 87}))
        points1.push(new ControlPoint({x: 25, y: 82, cx: 19, cy: 64}))

        // this.points = points;

        this.activePoly = this.addPolygon("test", COLOR.BLACK);
        this.activePoly.setPointSet(points);
        this.addPolygon("iris", COLOR.WHITE);
        this.polygonDict["iris"].setPointSet(points1);


        this.activePoly = this.polygons[0];


        this.redraw = true;
    }

    addPolygon(name, color = COLOR.BLACK) {
        let poly = new Polygon(this.rows, this.cols, color);
        this.polygons.push(poly);
        this.polygonDict[name] = poly;
        return poly;
    }

    setScale(scale = 1) {
        this.scale = scale;
        this.width = this.cols * scale;
        this.height = this.rows * scale;
        this.svgWidth = this.width + 2 * this.svgMargin * scale;
        this.svgHeight = this.height + 2 * this.svgMargin * scale;
    }


    synchronize(visualizer) {
        this.synchronizedVisualizer.push(visualizer);
        visualizer.synchronizedVisualizer.push(this);        
    }

    setFill(fill) {
        this.polygons.forEach(p => {
            p.fillPolygon = fill;
        });
    }

    showFillSteps(show) {
        this.polygons.forEach(p => {
            p.storeSteps = show;
        });
        this.__showFillSteps = show;
    }

    findNearesPoint(x, points) {
        
        let dif1 = 0;
        let dif2 = 0;

        let difs = [];

        points.forEach(p => {
            let dif = p - x;

            difs.push({p: p, dif: dif});
            // dif2 = dif1;
            // dif1 = dif;

            // if (dif1 > 0) {

            //     let abs1 = Math.abs(dif1);
            //     let abs2 = Math.abs(dif2);

            //     return abs1 > abs2 ? abs2 : abs1;
            // }            
        });

        difs.sort((a,b) => {return a.dif - b.dif});
        return difs[0].p;

    }

    getVisualizerDiv() {

        if (this.div == null) {
            var div = document.createElement("div");
            div.id = this.name;
            div.style.position = "relative";
    
            var canvas = document.createElement("canvas");
            canvas.classList.add("polygonCanvas");
            canvas.classList.add("border");
            canvas.classList.add("border-dark");
            let cMargin = this.svgMargin * this.scale;
            canvas.style.margin = `${cMargin}px ${cMargin}px ${cMargin}px ${cMargin}px`;
            canvas.width = this.width;
            canvas.height = this.height;    
            

            //var svg = document.createElement("svg");
            var svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
            svg.classList.add("controlSVG");
            svg.style.position = "absolute";
    
            div.appendChild(canvas);
            div.appendChild(svg);
    
            svg.style.top = canvas.offsetTop + this.trimY;
            svg.style.left = canvas.offsetLeft + this.trimX;
            svg.setAttribute("width", this.svgWidth);
            svg.setAttribute("height", this.svgHeight);

            this.div = div;
            this.svg = svg;
            this.canvas = canvas;

            let ctx = canvas.getContext("2d");
            ctx.fillStyle = "#000000";
            this.ctx = ctx;

            var fps = document.createElement("p");
            fps.id = "fps";
            fps.innerHTML = "Test";
            this.fps = fps;

            div.appendChild(fps);
        }


        return div;
    }

    init() {

        var this_ = this;

        d3.select(this.svg)
            .on("click", function(d) {
                this_.iActivePoly = (this_.iActivePoly + 1) % this_.polygons.length;
                this_.activePoly = this_.polygons[this_.iActivePoly];

                console.log("Active Poly: " + this_.iActivePoly);

                this_.initControls();
            })

        this.initControls();

        this.updatePolygon();
        

        this.display(this.activePoly);

        this.lastAnimationTime = Date.now();
        setTimeout(() => {
            this.animation();
        }, this.animationStep);

        setInterval(() => {
            this.fpsCalc();
        }, 1000);
    }

    dataToPoint(d) {
        return (d + this.svgMargin) * this.scale;
    }

    pointToData(p, controlSize = 0) {
        return Math.floor(p / this.scale + controlSize / 2 - this.svgMargin);
    }

    /**
     * Set the visibility of all control rectangles
     * @param {bool} visible If true alls controls rects get visible, invisible if false
     */
    setRectVisibility(visible) {
        if (visible) {
            this.controlRects.attr("visibility", "visible");
            this.synchronizedVisualizer.forEach(v => {
                v.controlRects.attr("visibility", "visible");
            });
        } else {
            this.controlRects.attr("visibility", "hidden");
            this.synchronizedVisualizer.forEach(v => {
                v.controlRects.attr("visibility", "hidden");
            });
        }

    }

    initControls() {

        var this_ = this;

        d3.select(this.svg)
            .on("mouseenter", function(event, d) { this_.setRectVisibility(true); })
            .on("mouseleave", function(event, d) { this_.setRectVisibility(false); })

        d3.select("body")
            .on("keydown", function(event) {
                if (event.key == "Control") {
                    this_.pressedCmd = true;
                    console.log("CMD down");
                }
            })
            .on("keyup", function(event) {
                if (event.key == "Control") {
                    this_.pressedCmd = false;
                    console.log("CMD up");
                }
            })


        // Clear all control elements
        d3.select(this.svg).selectAll(".pointRect").remove();
        d3.select(this.svg).selectAll(".curveRect").remove();
        d3.select(this.svg).selectAll(".geoCenter").remove();
        d3.select(this.svg).selectAll(".viewDirection").remove();

        d3.select(this.svg).selectAll(".viewLines").remove();

        d3.select(this.svg).selectAll(".viewLines").data(this.viewLines).enter()
            .append("line")
            .classed("viewLines", true)
            .style("stroke", "cyan")
            .attr("opacity", 0.5)    
        ;



        // Control to set the view direction
        d3.select(this.svg).selectAll(".viewDirection").data([this.viewDirection]).enter()
            .append("circle")
            .classed("viewDirection", true)
            .attr("fill", "white")
            .attr("fill-opacity", 0.3)
            .attr("stroke", "orange")
            .attr("stroke-width", 3)
            .attr("stroke-opacity", 0.8)
            // .on("mouseenter", function(event, d) { d3.select(this).attr("rx", this_.scale).attr("opacity", 0.7) })
            // .on("mouseleave", function(event, d) { d3.select(this).attr("rx", 0).attr("opacity", 0.3) })
            .call(this.drag_viewDirection())
            .on("click", function(event, d) {
                event.stopPropagation();
            })
            .on("dblclick", function(event, d) { 
                //console.log("DB Click Circle");
                //d.curve = 0; 
                d.x = 0;
                d.y = 0;
                
                this_.synchronizedVisualizer.forEach(v => {
                    v.viewDirection.x = 0;
                    v.viewDirection.y = 0;
                    v.updatePolygon(v.animationDuration);
                    v.updateControls({});
                });
                
                this_.updatePolygon(this_.animationDuration);
                this_.updateControls({}); 
            })
            ;


        // Control to set the polygon points
        d3.select(this.svg).selectAll(".pointRect").data(this.activePoly.points).enter()
            .append("rect")
            .classed("pointRect", true)
            .attr("rectType", "pointRect")
            .attr("fill", "red")
            .attr("opacity", 0.3)
            .on("mouseenter", function(event, d) { d3.select(this).attr("rx", this_.scale).attr("opacity", 0.7) })
            .on("mouseleave", function(event, d) { d3.select(this).attr("rx", 0).attr("opacity", 0.3) })
            .call(this.drag_controlRects())
            .on("click", function(event, d) {
                event.stopPropagation();
            })
            ;

        // Control to set the bezier curve points
        d3.select(this.svg).selectAll(".curveRect").data(this.activePoly.curvePoints).enter()
            .append("rect")
            .classed("curveRect", true)
            .attr("rectType", "curveRect")
            .attr("fill", "green")
            .attr("opacity", 0.3)
            .on("mouseenter", function(event, d) { d3.select(this).attr("rx", this_.scale).attr("opacity", 0.7) })
            .on("mouseleave", function(event, d) { d3.select(this).attr("rx", 0).attr("opacity", 0.3) })
            .call(this.drag_controlRects())
            .on("click", function(event, d) {
                event.stopPropagation();
            })
            .on("dblclick", function(event, d) { 
                d.curve = 0; 
                d.cy = false;
                d.cx = false;
                this_.updatePolygon(this_.animationDuration);
                this_.updateControls({}); 
            })
            ;

        // Control to change the center of the whole polygon
        d3.select(this.svg).selectAll(".geoCenter").data([this.activePoly.geoCenter]).enter()
            .append("rect")
            .classed("geoCenter", true)
            .attr("rectType", "geoCenter")
            .attr("fill", "blue")
            .attr("opacity", 0.3)
            .on("mouseenter", function(event, d) { d3.select(this).attr("rx", this_.scale).attr("opacity", 0.7) })
            .on("mouseleave", function(event, d) { d3.select(this).attr("rx", 0).attr("opacity", 0.3) })
            .call(this.drag_controlRects())
            .on("click", function(event, d) {
                console.log("Stop propagation")
                d3.event.stopPropagation();
            })
            ;

        this.controlRects = d3.select(this.svg).selectAll(".pointRect,.curveRect,.geoCenter")

        this.updateControls({});
    }

    setViewLines(x, y) {
        this.viewLines[0].x = false;
        this.viewLines[0].y = Math.floor(y);

        this.viewLines[1].x = false;
        this.viewLines[1].y = Math.floor(y + 1);

        this.viewLines[2].x = Math.floor(x);
        this.viewLines[2].y = false;

        this.viewLines[3].x = Math.floor(x + 1);
        this.viewLines[3].y = false;
    }

    drag_viewDirection() {
        var this_ = this;
        var deltaX = this.deltaX;
        var deltaY = this.deltaY;
        //var controlSize = this.controlSize;
        let m = this.svgMargin;
        var scale = this.scale;

        return d3.drag()
            .on("start", function(event, d) {
                var current = d3.select(this);
                deltaX = current.attr("cx") - event.x;
                deltaY = current.attr("cy") - event.y;

                this_.setRectVisibility(false);
            })
            .on("drag", function(event, d) {

                let newX = event.x + deltaX;
                let newY = event.y + deltaY;

                var current = d3.select(this);

                current
                .attr("cx", newX)
                .attr("cy", newY);

                d.x = this_.pointToData(newX, d.controlSize) - this_.cols / 2;
                d.y = this_.pointToData(newY, d.controlSize) - this_.rows / 2;

                this_.setViewLines(d.x, d.y);

                this_.synchronizedVisualizer.forEach(v => {
                    v.viewDirection.x = d.x;
                    v.viewDirection.y = d.y;
                    v.updatePolygon(v.animationDuration);
                    v.updateControls({});
                });

                this_.updatePolygon(this_.animationDuration);

                //console.log(points);
            })
            .on("end", function(event, d) {

                this_.updateControls({});

                this_.setRectVisibility(true);
            })
            ;
    }

    drag_controlRects() {

        var this_ = this;
        var deltaX = this.deltaX;
        var deltaY = this.deltaY;
        //var controlSize = this.controlSize;
        let m = this.svgMargin;
        var scale = this.scale;

        return d3.drag()
            .on("start", function(event, d) {
                //console.log("Drag start");
                var current = d3.select(this);
                deltaX = current.attr("x") - event.x;
                deltaY = current.attr("y") - event.y;

                d3.select(this).attr("stroke", "black");

                let showX = this_.pointToData(current.attr("x"), d.controlSize);
                let showY = this_.pointToData(current.attr("y"), d.controlSize);

                this_.activePoly.points.forEach(p => {
                    if (p != d) p.controlSize = 1;
                });
                this_.activePoly.curvePoints.forEach(p => {
                    if (p != d) p.controlSize = 1;
                });
                this_.activePoly.geoCenter.controlSize = 1;

                this_.controlRects.attr("opacity", 1);
                current.attr("opacity", 0.3);


                //console.log(`${showX} ${showY}`)
                this_.__showViewLines = true;
                this_.setViewLines(showX, showY);
                this_.updateControls({});

                //console.log("Start drag");
            })
            .on("drag", function(event, d) {

                var current = d3.select(this);

                var newX = event.x + deltaX;
                var newY = event.y + deltaY;

                
                
                current
                .attr("x", newX)
                .attr("y", newY);
                
                var xDat = this_.pointToData(newX, d.controlSize);
                var yDat = this_.pointToData(newY, d.controlSize);
                
                if (this_.pressedCmd) {
                    xDat = this_.findNearesPoint(xDat, this_.snapPointsX);
                    yDat = this_.findNearesPoint(yDat, this_.snapPointsY);
                    console.log(`New x ${xDat} y ${yDat}`)
                }
                //console.log(`${newX} | ${newY}`)

                //var curvePoint = current.classed("curveRect")
                var rectType = current.attr("rectType");
            
                if (rectType == "curveRect") {
                    d.cx = xDat;
                    d.cy = yDat;
                    d.curve = -1;

                    // console.log(`${d.cx} | ${d.cy}`)

                    this_.setViewLines(d.cx, d.cy);

                    this_.updateControls({curveRects: false});

                } else if (rectType == "pointRect") {
                    d.x = xDat;
                    d.y = yDat;

                    // console.log(`${d.x} | ${d.y}`)

                    this_.setViewLines(d.x, d.y);

                    this_.updateControls({pointRects: false});
                } else if (rectType == "geoCenter") {
                    let x = xDat;
                    let y = yDat;

                    let ap = this_.activePoly;

                    let difX = x - ap.geoCenter.x;
                    let difY = y - ap.geoCenter.y;


                    ap.points.forEach((p, i) => {
                        let cP = ap.curvePoints[i];
                        p.x += difX;
                        p.y += difY;
                        if (cP.curve != 0) {
                            cP.cx += difX;
                            cP.cy += difY;
                        }
                    });

                    this_.setViewLines(d.x, d.y);    
                    this_.updateControls({geoCenter: false});
                }
                
                
                this_.updatePolygon(this_.animationDuration);

                //console.log(points);
            })
            .on("end", function(event, d) {
                //console.log("Drag end");
                this_.__showViewLines = false;
                d3.select(this).attr("stroke", null);


                this_.activePoly.points.forEach(p => {
                    p.controlSize = this_.controlSize;
                });
                this_.activePoly.curvePoints.forEach(p => {
                    p.controlSize = this_.controlSize;
                });
                this_.activePoly.geoCenter.controlSize = this_.controlSize;

                this_.controlRects.attr("opacity", 0.3);
                
                this_.updateControls({});
                //console.log("End drag");
            })
            ;
    }


    updateControls( {pointRects = true, curveRects = true, geoCenter = true, viewDirection = true, viewLines = true} ) {

        let scale = this.scale;
        //let controlSize = this.controlSize;
        let points = this.activePoly.points;
        let curvePoints = this.activePoly.curvePoints;
        let this_ = this;
        let m = this.svgMargin;
        this.activePoly.calcGeoCenter();


        // Update the position of the view direction circle
        if (viewDirection) {
            d3.select(this.svg).select(".viewDirection").data([this.viewDirection])
                .attr("cx", function(d) { return (d.x + 0.5 + m + this_.cols / 2 - d.controlSize / 2) * scale})
                .attr("cy", function(d) { return (d.y + 0.5 + m + this_.rows / 2 - d.controlSize / 2) * scale})
                .attr("r", (d) => (d.controlSize * scale))
                ;
        }

        // Update the position of the polygon points
        if (pointRects) {
            d3.select(this.svg).selectAll(".pointRect").data(points)
            .attr("width", function(d) { return d.controlSize * scale} )
            .attr("height", function(d) { return d.controlSize * scale} )
            .attr("x", function(d) { return (d.x + 0.5 + m - d.controlSize / 2) * scale})
            .attr("y", function(d) { return (d.y + 0.5 + m - d.controlSize / 2) * scale})
            ;
        }

        // Update the position of the curve points    
        if (curveRects) {
            d3.select(this.svg).selectAll(".curveRect").data(curvePoints)
            .attr("width", function(d) { return d.controlSize * scale} )
            .attr("height", function(d) { return d.controlSize * scale} )
            .attr("x", function(d, i) {
                let x = d.cx;
                let y = d.cy;
                if (x == false || y == false) {
                    x = (points[i].x + points[(i + 1) % points.length].x) / 2;
                    y = (points[i].y + points[(i + 1) % points.length].y) / 2;
                }
                return (x + 0.5 + m - d.controlSize / 2) * scale;
            })
            .attr("y", function(d, i) { 
                let x = d.cx;
                let y = d.cy;
                if (x == false || y == false) {
                    x = (points[i].x + points[(i + 1) % points.length].x) / 2;
                    y = (points[i].y + points[(i + 1) % points.length].y) / 2;
                }
                return (y + 0.5 + m - d.controlSize / 2) * scale ;    
            })
            ;
        }
        
        // Update the position of the polygon center
        if (geoCenter) {
            d3.select(this.svg).selectAll(".geoCenter").data([this.activePoly.geoCenter])
            .attr("width", function(d) { return d.controlSize * scale} )
            .attr("height", function(d) { return d.controlSize * scale} )
            .attr("x", function(d) { return (d.x + 0.5 + m - d.controlSize / 2) * scale})
            .attr("y", function(d) { return (d.y + 0.5 + m - d.controlSize / 2) * scale})
            ;
        }

        if (viewLines) {
            //Update the view lines
            d3.select(this.svg).selectAll(".viewLines").data(this.viewLines)
                .attr("x1", function(d) { 
                    //if (!d.y) return d.x * scale ;
                    if (!d.y) return this_.dataToPoint(d.x);
                    else if (!d.x) return this_.dataToPoint(0);
                })
                .attr("x2", function(d) { 
                    if (!d.y) return this_.dataToPoint(d.x);
                    else if (!d.x) return this_.dataToPoint(this_.cols);
                })
                .attr("y1", function(d) { 
                    if (!d.y) return this_.dataToPoint(0);
                    else if (!d.x) return this_.dataToPoint(d.y);
                })
                .attr("y2", function(d) { 
                    if (!d.y) return this_.dataToPoint(this_.rows);
                    else if (!d.x) return this_.dataToPoint(d.y);
                })
                .attr("visibility", function(d) { return this_.__showViewLines ? "visible" : "hidden" });
        }

        

    ;
    }


    updatePolygon(animationTime = 0) {


        // console.log(`View: x: ${this.viewDirection.x} | y: ${this.viewDirection.y}`);

        this.polygons.forEach(poly => {            
            poly.clearPoints();
            for (let i = 0; i < poly.points.length; i++) {
                const p = poly.points[i];
                const cp = poly.curvePoints[i];

                poly.addPoint(p.x, p.y, cp.cx, cp.cy, this.viewDirection, p.viewInfluence);
                //poly.addPoint(p.x, p.y, p.cx, p.cy);
            }
           poly.setAnimationDuration(animationTime);

        });

        this.snapPointsX = [];
        this.snapPointsY = [];

        this.activePoly.points.forEach(p => {
            this.snapPointsX.push(p.x);
            this.snapPointsY.push(p.y);
        });
        this.activePoly.curvePoints.forEach(p => {
            if (p.cx != false) this.snapPointsX.push(p.cx);
            if (p.cy != false) this.snapPointsY.push(p.cy);
        });
        this.snapPointsX.sort((a, b) => {return a-b});
        this.snapPointsY.sort((a, b) => {return a-b});

        this.redraw = true;
    }


    
    fpsCalc() {
        this.fps.innerHTML = Math.round(this.fpsCounter);
        this.fpsCounter = 0;
    }


    animation() {       
        
        let time = Date.now() - this.lastAnimationTime;
        this.lastAnimationTime = Date.now();

        if (this.redraw) {

            this.polygons.forEach(poly => {            
                // this.redraw = !
                poly.animatePolygon(time);
                poly.drawPolygon();
                //this.display(this.activePoly);
                this.clear();
                this.displayAll();
    
            });

            // this.redraw = !this.activePoly.animatePolygon(time);
            // this.activePoly.drawPolygon();
            // //this.display(this.activePoly);
            // this.clear();
            // this.displayAll();
        }
        
        if (this.__showFillSteps) {
            setTimeout(() => {
                this.displayStep(this.polygons[0], 0);
            }, 5);
        } else {
            setTimeout(() => {
                this.animation();
            }, 5);
        }



        this.fpsCounter += 1;
    }

        
    /**
     * 
     * @param {Polygon} p 
     * @param {*} stepIndex 
     */
    displayStep(p, stepIndex) {

        //printPolygon(p);
        this.clear();

        let buffer = p.steps[stepIndex];
        this.displayBuffer(buffer, p.gBuf.width, p.color);

        let t = this;
        stepIndex = (stepIndex + 1) % p.steps.length;

        if (stepIndex > 0) {
            setTimeout(() => {
               this.displayStep(p, stepIndex);
            }, 1);
        } else {
            setTimeout(() => {
                this.animation();
            }, 5);
        }


    }


    clear() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    }

    displayAll() {

        this.polygons.forEach(p => {
            this.display(p);
        });
    }

    /**
     * 
     * @param {Polygon} p 
     */
    display(p) {

        // printPolygon(p);
        
        let start = performance.now()

        for (let i = 0; i < p.gBuf.buffer.length; i++) {
            const e = p.gBuf.buffer[i];
            
            for (let x = 0; x < 8; x++) {

                let shift = (1 << x);
                let res = (e & shift); 
                if (res > 0) {

                    let xCord = ((i * 8) % p.gBuf.width) + x;
                    let yCord = Math.floor(i * 8 / p.gBuf.width); 
                    this.drawPixel(xCord, yCord, p.color);
                }
            }
            
        }

        let end = performance.now();

        //console.log("Draw polygon took " + (end - start) + " milliseconds.")
    }   


    displayBuffer(buffer, width, color) {
        for (let i = 0; i < buffer.length; i++) {
            const e = buffer[i];
            
            for (let x = 0; x < 8; x++) {

                let shift = (1 << x);
                let res = (e & shift); 
                if (res > 0) {

                    let xCord = ((i * 8) % width) + x;
                    let yCord = Math.floor(i * 8 / width); 
                    this.drawPixel(xCord, yCord, color);
                }
            }
            
        }
    }

    drawPixel(x, y, color) {
        this.ctx.fillStyle = color == COLOR.BLACK ? "#000000" : "#DDDDDD";
        this.ctx.fillRect(x * this.scale, y * this.scale, this.scale, this.scale);
        this.ctx.fillRect(x * this.scale, y * this.scale, this.scale, this.scale);
    }





}

