const FLOODFILL = {
    SOURCE_XM : 1,
    SOURCE_XP : 2,
    SOURCE_YM : 3,
    SOURCE_YP : 4
}


class Point {
    constructor( {x = 0, y = 0, cx = false, cy = false, viewInfluence = 100 } ) {

        this.curve = cx != false || cy != false ? -1 : 0;

        this.x = x;
        this.y = y;

        this.cx = cx;
        this.cy = cy;

        this.viewInfluence = viewInfluence;
    }

    static clone(p) {
        let c = new Point({
            x: p.x,
            y: p.y,
            cx: p.cx,
            cy: p.cy,
            viewInfluence: p.viewInfluence 
        });

        return c;
    }
}

class ControlPoint extends Point {
    constructor( {x = 0, y = 0, cx = false, cy = false, viewInfluence = 100, controlSize = 3 } ) {
        super({x: x, y: y, cx: cx, cy: cy, viewInfluence: viewInfluence})
        this.controlSize = controlSize;
    }

    static clone(p) {
        let c = new ControlPoint({
            x: p.x,
            y: p.y,
            cx: p.cx,
            cy: p.cy,
            viewInfluence: p.viewInfluence,
            controlSize: p.controlSize
        });

        return c;

    }
}



class Polygon {

    static maxPoints = 16;
    static maxFillPoints = 2;

    static MATRIX_ORIENTATION = true;


    constructor(rows, cols, color) {

        this.storeSteps = false;
        this.steps = new Array();

        this.rows = rows;
        this.cols = cols;

        this.gBuf = new GFX_Buffer(rows, cols);

        this.color = color;

        this.pointCount = 0;
        this.fillPointCount = 0;

        this.fillPolygon = true;

        this.geoCenter = new ControlPoint({});

        this.points = [];
        this.curvePoints = [];

        this.origin = new Array();
        this.current = new Array();
        this.target = new Array();

        for (let i = 0; i < Polygon.maxPoints; i++) {
            this.origin.push(new Point({}));
            this.current.push(new Point({}));
            this.target.push(new Point({}));
        }


        this.fillPointsX = new Array(Polygon.maxFillPoints);
        this.fillPointsY = new Array(Polygon.maxFillPoints);

        // End time of the animation
        this.animationEnd;

        this.currentAnimation;

        this.lastAnimateMillis;
    }

    setRotation(r) {
        this.gBuf.setRotation(r);
    }


    copyGfxBuffer(targetBuf) {
        targetBuf = new Uint8Array(...this.buffer);
    }

    combineBuffer(targetBuf, color) {
        if (color == COLOR.WHITE) {
            for (let i = 0; i < this.gBuf.buffer.length; i++) {
                const b = this.gBuf.buffer[i];
                targetBuf[i] |= b;
            }
        }
        else {

            for (let i = 0; i < this.gBuf.buffer.length; i++) {
                const b = this.gBuf.buffer[i];
                targetBuf[i] &= ~b;
            }
        }
    }


    clearPoints() {
        for (let i = 0; i < this.pointCount; i++) {            
            this.origin[i].x = this.current[i].x;
            this.origin[i].y = this.current[i].y;
            this.origin[i].c = this.current[i].c;
            this.origin[i].cx = this.current[i].cx;
            this.origin[i].cy = this.current[i].cy;
        }

        this.pointCount = 0;
        this.fillPointCount = 0;
        this.currentAnimation = 0;
        this.animationEnd = 0;
    }


    setPointSet(points) {
        this.points = points;
        points.forEach(p => {
            this.curvePoints.push(ControlPoint.clone(p));
        });
    }

    addPoint(x, y, cx = false, cy = false, viewPoint = null, viewInfluence = 0) {
        
        if (viewPoint != null && viewInfluence != 0) {
            
            let xInfluence = Math.floor(viewPoint.x * viewInfluence / 100);
            let yInfluence = Math.floor(viewPoint.y * viewInfluence / 100);

            this.target[this.pointCount].x = x + xInfluence;
            this.target[this.pointCount].y = y + yInfluence;
            this.target[this.pointCount].c = (cx == false && cy == false) ? 0 : -1;
            this.target[this.pointCount].cx = cx + xInfluence;
            this.target[this.pointCount].cy = cy + yInfluence;

        } else {
            this.target[this.pointCount].x = x;
            this.target[this.pointCount].y = y;
            this.target[this.pointCount].c = (cx == false && cy == false) ? 0 : -1;
            this.target[this.pointCount].cx = cx;
            this.target[this.pointCount].cy = cy;
        }


        this.pointCount++;

        this.calcGeoCenter();
    }


    getArea() {

        let sum = 0;

        for (let i = 0; i < this.points.length - 1; i++) {
            const p = this.points[i];
            const p1 = this.points[(i + 1) % this.points.length];

            sum += p.x * p1.y - p1.x * p.y;            
        }

        sum /= 2;
        return sum;
    }

    calcGeoCenter() {
        let area = this.getArea();

        let xs = 0;
        let ys = 0;

        let xSum = 0;
        let ySum = 0;

        // for (let i = 0; i < this.points.length - 1; i++) {
        //     const p = this.points[i];
        //     const p1 = this.points[(i + 1) % this.points.length];

        //     xSum += (p.x + p1.x) * (p.x * p1.y - p1.x * p.y);      
        //     ySum += (p.x + p1.y) * (p.x * p1.y - p1.x * p.y);      
        // }

        // xs = xSum / (6 * area);
        // ys = ySum / (6 * area);

        let maxX = 0;
        let minX = Infinity;
        let maxY = 0;
        let minY = Infinity;        

        for (let i = 0; i < this.points.length; i++) {
            const p = this.points[i];

            maxX = Math.max(maxX, p.x);   
            minX = Math.min(minX, p.x);   
            maxY = Math.max(maxY, p.y);   
            minY = Math.min(minY, p.y);   
        }

        this.geoCenter.x = (maxX + minX) / 2;
        this.geoCenter.y = (maxY + minY) / 2;

        return this.geoCenter;
    }

    drawPolygon() {

        //TODO: Pixel die außerhalb des Displays liegen, werden derzeit auf min. 0 und max. Breite bzw. Höhe des Displays gesetzt. 
        //Dadurch sind teilweise zu große Linien am Rand des Bildschirms.
        //Lösung: Merken, an welchen Stellen die Linien die Bildschirmgrenzen durchstoßen und nur dort eine Linie zeichnen

        this.steps = [];

        let len = this.pointCount;

        this.gBuf.clearBuffer();

        if (this.pointCount == 0)
            return;

        //Serial.println("Draw Pol");
        //Serial.print("Points:");
        //Serial.println(pointCount);

        // Draw the outline
        for (let i = 0; i < len; i++) {

            let curve = this.current[i].c;

            // If the curve value == 0, draw a straight line
            if (curve == 0) {
                let x0 = this.current[i].x;
                let x1 = this.current[(i + 1) % len].x;
                let y0 = this.current[i].y;
                let y1 = this.current[(i + 1) % len].y;

                this.gBuf.drawLine(x0, y0, x1, y1, COLOR.WHITE);
            }

            // If the curve of the line is > 0
            // a cubic bezier curve is drawn between p(i) and p(i + 1)
            // The curve is based on the slope in these two points and
            // is calculated by taking the theoretic intersection point of the lines from p(i - 1) to p(i) and from p(i + 1) to p(i + 2)
            // A value of 100 means that the calculated intersection point is taken as reference for the bezier curve,
            // a value between 0 and 100 takes the percentual part from the base points to this intersection point.
            else if (curve > 0) {

                //TODO: einbauen, dass bei curve == 100 keine kubische sondern eine quadratische Beziercurve genommen wird.

                let x0 = this.current[i].x;
                let x1 = this.current[(i + 1) % len].x;
                let x2 = this.current[(i + 2) % len].x;
                let x3 = this.current[(i + len - 1) % len].x;

                let y0 = this.current[i].y;
                let y1 = this.current[(i + 1) % len].y;
                let y2 = this.current[(i + 2) % len].y;
                let y3 = this.current[(i + len - 1) % len].y;

                // The bezier curve connects to lines g and h
                // g is the line from the point before the current point to the current point
                // h is the line from the point after the next point to the next point

                // gx = x3 + g * (x0 - x3)
                // gy = y3 + g * (y0 - y3)

                // hx = x2 + g * (x1 - x2)
                // hy = y2 + g * (y1 - y2)

                // The following term calculates the value of the intersection point between the extended lines of g and h

                let g1 = (y1 - y2) * x2 - (x1 - x2) * y2 - (y1 - y2) * x3 + (x1 - x2) * y3;
                let g2 = (x0 - x3) * (y1 - y2) - (y0 - y3) * (x1 - x2);
                let g = g1 / g2;

                let xS = x3 + g * (x0 - x3);
                let yS = y3 + g * (y0 - y3);

                // console.log("i: " + i);
                // console.log("g: " + g);
                // console.log(xS, yS);

                let xb0 = this.current[i].x;
                let xb3 = this.current[(i + 1) % len].x;

                let xb1 = xb0 + (xS - xb0) * curve / 100.0;
                let xb2 = xb3 + (xS - xb3) * curve / 100.0;

                let yb0 = this.current[i].y;
                let yb3 = this.current[(i + 1) % len].y;

                let yb1 = yb0 + (yS - yb0) * curve / 100;
                let yb2 = yb3 + (yS - yb3) * curve / 100;

                let t = 0.0;

                let intervall = 0.05;
                let lastXbp = -1;
                let lastYbp = -1;

                while (t <= 1.0) {

                    let xbp = ((-xb0 + 3 * xb1 - 3 * xb2 + xb3) * t * t * t + (3 * xb0 - 6 * xb1 + 3 * xb2) * t * t + (-3 * xb0 + 3 * xb1) * t + xb0 + 0.5); // +0.5 for rounding

                    let ybp = ((-yb0 + 3 * yb1 - 3 * yb2 + yb3) * t * t * t + (3 * yb0 - 6 * yb1 + 3 * yb2) * t * t + (-3 * yb0 + 3 * yb1) * t + yb0 + 0.5);

                    // Intervall anpassen je nachdem ob das neue Pixel weiter entfernt ist als 1
                    if (lastXbp == -1)
                        lastXbp = xbp;
                    if (lastYbp == -1)
                        lastYbp = ybp;

                    if (Math.abs(lastXbp - xbp) > 1 || Math.abs(lastYbp - ybp) > 1) {
                        intervall = intervall / 2;
                        t -= intervall;
                        //console.log("Decreased intervall to " + intervall);
                        continue;
                    }

                    lastXbp = xbp;
                    lastYbp = ybp;

                    // this->gBuf->drawPixel(xbp, ybp, 2);
                    this.gBuf.drawPixel(xbp, ybp, WHITE);

                    t += intervall;
                }

                // drawPixel(xS, yS, 3);
            }

            // Quadratic bezier curve
            // If the current point p(i) has the curve mode == -1
            // than a quadratic bezier line is drawn to p(i + 2)  with aid of the point p(i + 1)
            // https://de.wikipedia.org/wiki/B%C3%A9zierkurve
            else if (curve == -1) {

                let x0 = this.current[i].x;
                let x1 = this.current[i].cx;
                let x2 = this.current[(i + 1) % len].x;

                let y0 = this.current[i].y;
                let y1 = this.current[i].cy;
                let y2 = this.current[(i + 1) % len].y;


                let t = 0.0;

                let intervall = 0.05;
                let lastXp = -1;
                let lastYp = -1;

                while (t <= 1.0) {

                    let xp = (x0 - 2 * x1 + x2) * (t * t) + (-2 * x0 + 2 * x1) * t + x0;
                    let yp = (y0 - 2 * y1 + y2) * (t * t) + (-2 * y0 + 2 * y1) * t + y0;

                    // Intervall anpassen je nachdem ob das neue Pixel weiter entfernt ist als 1
                    if (lastXp == -1)
                        lastXp = xp;
                    if (lastYp == -1)
                        lastYp = yp;

                    if (Math.abs(lastXp - xp) > 1 || Math.abs(lastYp - yp) > 1) {
                        intervall = intervall / 2;
                        t -= intervall;
                        //console.log("Decreased intervall to " + intervall);
                        continue;
                    }

                    lastXp = xp;
                    lastYp = yp;

                    //this->gBuf->drawPixel(Math.round(xp), Math.round(yp), 2);
                    this.gBuf.drawPixel(xp, yp, COLOR.WHITE);

                    t += intervall;
                }

                //i++;

                // drawPixel(x1, y1, 3);
            }

            // Cubic bezier curve
            // If the current point p(i) has the curve mode == -2
            // than a cubic bezier line is drawn to p(i + 3) through with aid of the points p(i + 1) and p(i + 2)
            // https://de.wikipedia.org/wiki/B%C3%A9zierkurve
            else if (curve == -2) {

                let xb0 = this.current[i].x;
                let xb1 = this.current[(i + 1) % len].x;
                let xb2 = this.current[(i + 2) % len].x;
                let xb3 = this.current[(i + 3) % len].x;

                let yb0 = this.current[i].y;
                let yb1 = this.current[(i + 1) % len].y;
                let yb2 = this.current[(i + 2) % len].y;
                let yb3 = this.current[(i + 3) % len].y;

                let t = 0.0;

                let intervall = 0.05;
                let lastXbp = -1;
                let lastYbp = -1;

                while (t <= 1.0) {

                    let xbp = (-xb0 + 3 * xb1 - 3 * xb2 + xb3) * t * t * t + (3 * xb0 - 6 * xb1 + 3 * xb2) * t * t + (-3 * xb0 + 3 * xb1) * t + xb0;
                    let ybp = (-yb0 + 3 * yb1 - 3 * yb2 + yb3) * t * t * t + (3 * yb0 - 6 * yb1 + 3 * yb2) * t * t + (-3 * yb0 + 3 * yb1) * t + yb0;

                    // Intervall anpassen je nachdem ob das neue Pixel weiter entfernt ist als 1
                    if (lastXbp == -1)
                        lastXbp = xbp;
                    if (lastYbp == -1)
                        lastYbp = ybp;

                    if (Math.abs(lastXbp - xbp) > 1 || Math.abs(lastYbp - ybp) > 1) {
                        intervall = intervall / 2;
                        t -= intervall;
                        //console.log("Decreased intervall to " + intervall);
                        continue;
                    }

                    lastXbp = xbp;
                    lastYbp = ybp;

                    //this->gBuf->drawPixel(Math.round(xbp), Math.round(ybp), 2);
                    this.gBuf.drawPixel(xbp, ybp, WHITE);

                    t += intervall;
                }

                i += 2;

                // drawPixel(xb1, yb1, 3);
                // drawPixel(xb2, yb2, 3);
            }
        }

        // Fill the polygon
        
        if (!this.fillPolygon) return;
        //return;

        let oldFillPointCount = this.fillPointCount;

        // If there are no specific filling points, calculate one
        if (this.fillPointCount == 0) {

            //Serial.println("Calc fill point");

            let maxY = 0;
            let minY = this.rows - 1;

            //TODO: max und min Y nur beachten, wenn y innerhalb des Displays ist 

            // Get min and max y of the polygon
            for (let i = 0; i < len; i++) {
                maxY = Math.max(this.current[i].y, maxY);
                minY = Math.min(this.current[i].y, minY);
            }
           

            maxY = Math.min(maxY, this.rows - 2);
            minY = Math.max(minY, 1);

            
            
            // and take the half of it
            let startY = minY + (maxY - minY) / 2;
            let start = false;
            
            //console.log(`${startY} ${maxY} ${minY}`);

            // Search for the first not empty pixel, after that take the first empty pixel as flood fill start point
            for (let c = 0; c < this.cols - 1; c++) {
                if (!start) {
                    if (this.gBuf.getPixel(c, startY) != 0)
                        start = true;
                }
                else {
                    if (this.gBuf.getPixel(c, startY) == 0) {
                        this.fillPointsX[this.fillPointCount] = c;
                        this.fillPointsY[this.fillPointCount++] = startY;

                        //floodPoints.push(new Point(c, startY));
                        break;
                    }
                }
            }
        }

        // Flood fill by the given flood points
        for (let i = 0; i < this.fillPointCount; i++) {
            let fpX = this.fillPointsX[i];
            let fpY = this.fillPointsY[i];

            //Serial.print("Flood fill with");
            //Serial.print(fpX);
            //Serial.print(",");
            //Serial.println(fpY);

            this.floodFill(fpX, fpY, 0);
            //this->gBuf->drawPixel(fp.x, fp.y, 4);
        }

        this.fillPointCount = oldFillPointCount;

        return;
    }


    floodFill(x, y, source) {
        if (x >= this.gBuf.width || x < 0 || y < 0 || y >= this.gBuf.height) {
            // Serial.print("Flood return --> ");
            // Serial.print(x);
            // Serial.print("|");
            // Serial.println(y);
            return;
        }

        let c = this.gBuf.getPixel(x, y);

        // Serial.print(x);
        // Serial.print("|");
        // Serial.print(y);
        // Serial.print("|");
        // Serial.println(c);
        // delay(5);

        if (this.storeSteps) this.steps.push(this.gBuf.buffer.slice());

        // updateDisplay();

        if (!c) {

            let index = this.gBuf.getBufferByteIndex(x, y);
            // Serial.print("Index: ");
            // Serial.println(index);
            // Serial.print("Value:");
            // Serial.println(this->gBuf->buffer[index]);

            if (this.gBuf.buffer[index] == 0x00) {
                this.gBuf.buffer[index] = 0xFF;

                let r = this.gBuf.rotation;

                if ( (Polygon.MATRIX_ORIENTATION && (r == 0 || r == 2)) ||
                    (!Polygon.MATRIX_ORIENTATION && (r == 1 || r == 3))
                    )
                    {
                        // For this rotation a byte is a 8-pixel-bar parallel to the x axis.

                        // We have to check if the upper or lower byte already contains a pixel,
                        // if this is the case, we have to flood fill every upper / lower pixel of the bar,
                        // to avoid small unfilled areas

                        let iTop = this.gBuf.getBufferByteIndex(x, y - 1);
                        let iBot = this.gBuf.getBufferByteIndex(x, y + 1);

                        let top = this.gBuf.buffer[iTop];
                        let bot = this.gBuf.buffer[iBot];

                        if (source != FLOODFILL.SOURCE_XM)
                            this.floodFill(Math.floor(x / 8) * 8 + 8, y, FLOODFILL.SOURCE_XP); // Right pixel
                        if (source != FLOODFILL.SOURCE_XP)
                            this.floodFill(Math.floor(x / 8) * 8 - 1, y, FLOODFILL.SOURCE_XM); // Left pixel

                        if (bot == 0x00) {
                            if (source != FLOODFILL.SOURCE_YM)
                                this.floodFill(Math.floor(x / 8) * 8, y + 1, FLOODFILL.SOURCE_YP); // Left Top
                        }
                        else if (bot != 0xFF) {
                            for (let i = 0; i < 8; i++)
                                this.floodFill(Math.floor(x / 8) * 8 + i, y + 1, FLOODFILL.SOURCE_YP);
                        }

                        if (top == 0x00) {
                            if (source != FLOODFILL.SOURCE_YP)
                                this.floodFill(Math.floor(x / 8) * 8, y - 1, FLOODFILL.SOURCE_YM); // Left Bot
                        }
                        else if (top != 0xFF) {
                            for (let i = 0; i < 8; i++)
                                this.floodFill(Math.floor(x / 8) * 8 + i, y - 1, FLOODFILL.SOURCE_YM);
                        }
                        //break;
                    }
                else if ( (Polygon.MATRIX_ORIENTATION && (r == 1 || r == 3)) ||
                        (!Polygon.MATRIX_ORIENTATION && (r == 0 || r == 2)))
                    {
                        // For this rotation a byte is a 8-pixel-bar parallel to the y axis.

                        // We have to check if the left or right byte already contains a pixel,
                        // if this is the case, we have to flood fill every left / right pixel of the bar,
                        // to avoid small unfilled areas

                        let iLeft = this.gBuf.getBufferByteIndex(x - 1, y);
                        let iRight = this.gBuf.getBufferByteIndex(x + 1, y);

                        let left = this.gBuf.buffer[iLeft];
                        let right = this.gBuf.buffer[iRight];

                        if (source != FLOODFILL.SOURCE_YM)
                            this.floodFill(x, Math.floor(y / 8) * 8 + 8, FLOODFILL.SOURCE_YP); // Top pixel
                        if (source != FLOODFILL.SOURCE_YP)
                            this.floodFill(x, Math.floor(y / 8) * 8 - 1, FLOODFILL.SOURCE_YM); // Bot pixel

                        if (right == 0x00) {
                            if (source != FLOODFILL.SOURCE_XM)
                                this.floodFill(x + 1, Math.floor(y / 8) * 8, FLOODFILL.SOURCE_XP); // Top right
                        }
                        else if (right != 0xFF) {
                            for (let i = 0; i < 8; i++)
                                this.floodFill(x + 1, Math.floor(y / 8) * 8 + i, FLOODFILL.SOURCE_XP);
                        }

                        if (left == 0x00) {
                            if (source != FLOODFILL.SOURCE_XP)
                                this.floodFill(x - 1, Math.floor(y / 8) * 8, FLOODFILL.SOURCE_XM); // Top left
                        }
                        else if (left != 0xFF) {
                            for (let i = 0; i < 8; i++)
                                this.floodFill(x - 1, Math.floor(y / 8) * 8 + i, FLOODFILL.SOURCE_XM);
                        }
                        //break;
                    }
                
            }
            else {
                this.gBuf.drawPixel(x, y, COLOR.WHITE);
                // this->gBuf->drawPixel(x, y, 3);

                if (source != FLOODFILL.SOURCE_YM)
                    this.floodFill(x, y + 1, FLOODFILL.SOURCE_YP);
                if (source != FLOODFILL.SOURCE_YP)
                    this.floodFill(x, y - 1, FLOODFILL.SOURCE_YM);
                if (source != FLOODFILL.SOURCE_XM)
                    this.floodFill(x + 1, y, FLOODFILL.SOURCE_XP);
                if (source != FLOODFILL.SOURCE_XP)
                    this.floodFill(x - 1, y, FLOODFILL.SOURCE_XM);

                this.cornerFill(x + 1, y + 1);
                this.cornerFill(x + 1, y - 1);
                this.cornerFill(x - 1, y + 1);
                this.cornerFill(x - 1, y - 1);
            }
        }
    }
    

    cornerFill(x, y)
    {
        //return;

        if (x >= this.gBuf.width || x < 0 || y < 0 || y >= this.gBuf.height)
            return;

        let c = this.gBuf.getPixel(x, y);
        let p1 = this.gBuf.getPixel(x + 1, y);
        let p2 = this.gBuf.getPixel(x - 1, y);
        let p3 = this.gBuf.getPixel(x, y + 1);
        let p4 = this.gBuf.getPixel(x, y - 1);

        let borderCount = 0;
        if (p1)
            borderCount++;
        if (p2)
            borderCount++;
        if (p3)
            borderCount++;
        if (p4)
            borderCount++;

        if (!c && (borderCount > 3))
        {
            this.gBuf.drawPixel(x, y, COLOR.WHITE);

            if (this.storeSteps) this.steps.push(this.gBuf.buffer.slice());


            this.floodFill(x, y + 1, 1);
            this.floodFill(x, y - 1, 2);
            this.floodFill(x + 1, y, 3);
            this.floodFill(x - 1, y, 4);

            this.cornerFill(x + 1, y + 1);
            this.cornerFill(x + 1, y - 1);
            this.cornerFill(x - 1, y + 1);
            this.cornerFill(x - 1, y - 1);
        }
    }

    setAnimationDuration(ms) {
        this.animationEnd = ms;
    }

    animatePolygon(ms) {
        let animFinished = false;

        if (this.animationEnd == 0 ||
            this.animationEnd <= this.currentAnimation + ms ||
            ms == 0) {
            for (let i = 0; i < this.pointCount; i++)
            {
                this.current[i].x = this.target[i].x;
                this.current[i].y = this.target[i].y;
                this.current[i].c = this.target[i].c;
                this.current[i].cx = this.target[i].cx;
                this.current[i].cy = this.target[i].cy;
    
                this.origin[i].x = this.target[i].x;
                this.origin[i].y = this.target[i].y;
                this.origin[i].c = this.target[i].c;
                this.origin[i].cx = this.target[i].cx;
                this.origin[i].cy = this.target[i].cy;
    
                this.animationEnd = 0;
                this.currentAnimation = 0;
    
                animFinished = true;
            }
        }
        else {
            this.currentAnimation += ms;
            // let factor = this.currentAnimation / this.animationEnd;
            //1f - Mathf.Exp((Mathf.Log(1f - 0.99f) / positionLerpTime) * Time.deltaTime);
            let lerpPercentage = 1 - Math.exp((Math.log(1 - 0.99) / this.animationEnd) * ms);
    
            for (let i = 0; i < this.pointCount; i++)
            {
                let o = this.origin[i];
                let oN = this.origin[(i + 1) % this.pointCount];

                let c = this.current[i];
                let cN = this.current[(i + 1) % this.pointCount];

                let t = this.target[i];
                let tN = this.target[(i + 1) % this.pointCount]


    
                if (t.c == -1 && o.c != -1) {
                    o.cx = (o.x + oN.x) / 2;
                    o.cy = (o.y + oN.y) / 2;
                    o.c = -1;
                    c.c = -1;

                    c.cx = o.cx;
                    c.cy = o.cy;
                }
                else if (t.c == -1 && o.c == -1) {
                    c.c = -1;
                }
                else if (t.c == 0 && o.c == -1) {
                    t.c = -1;
                    c.c = -1;
                    o.c = -1;
                    t.cx = (t.x + tN.x) / 2;
                    t.cy = (t.y + tN.y) / 2;
    
                    // Serial.print("Adapt target"); Serial.println("------------------------");
                    // Serial.print("targetX0: "); Serial.println(this->targetX[i]);
                    // Serial.print("targetX1: "); Serial.println(this->targetX[(i + 1) % this->pointCount]);
                    // Serial.print("targetY0: "); Serial.println(this->targetY[i]);
                    // Serial.print("targetY1: "); Serial.println(this->targetY[(i + 1) % this->pointCount]);
                    // Serial.print("targetCX: "); Serial.println(this->targetCX[i]);
                    // Serial.print("targetCY: "); Serial.println(this->targetCY[i]);
                    // Serial.print("originCX: "); Serial.println(this->originCX[i]);
                    // Serial.print("originCY: "); Serial.println(this->originCY[i]);
    
                }
                else if (this.target[i].c > 0) {
                    c.c = o.c + (t.c - o.c) * factor;
                }
    
                // c.x = o.x + (t.x - o.x) * factor;
                // c.y = o.y + (t.y - o.y) * factor;

                // c.cx = o.cx + (t.cx - o.cx) * factor;
                // c.cy = o.cy + (t.cy - o.cy) * factor;
                
                c.x = c.x + (t.x - c.x) * lerpPercentage;
                c.y = c.y + (t.y - c.y) * lerpPercentage;

                c.cx = c.cx + (t.cx - c.cx) * lerpPercentage;
                c.cy = c.cy + (t.cy - c.cy) * lerpPercentage;

                // let b = 0.5;
                // if (Math.abs(c.x - t.x) <= b) c.x = t.x;
                // if (Math.abs(c.y - t.y) <= b) c.y = t.y;
                // if (Math.abs(c.cx - t.cx) <= b) c.cx = t.cx;
                // if (Math.abs(c.cy - t.cy) <= b) c.cy = t.cy;
    
            }
        }
    
        return animFinished;
    }

}


