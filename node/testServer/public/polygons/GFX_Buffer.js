
function gfx_swap(a, b) {
    t = a;
    a = b;
    b = a;
}

class GFX_Buffer {
    constructor(rows, cols) {
        this.height = rows;
        this.width = cols;

        this.rotation = 0;

        this.clearBuffer();
    }

    clearBuffer() {
        this.buffer = new Uint8Array(this.height * Math.floor((this.width + 7) / 8))
    }

    setRotation(r) {
        this.rotation = r
    }

    /**
     * 
     * @param {Integer} x Column of display -- 0 at left to (screen width - 1) at right.
     * @param {Integer} y Row of display -- 0 at top to (screen height -1) at bottom.
     * @param {Integer} color Pixel color, one of: COLOR.WHITE, COLOR.BLACK or COLOR.INVERSE
     */
    drawPixel(x, y, color) {

        x = Math.floor(x + 0.5);
        y = Math.floor(y + 0.5);

        x = Math.min(Math.max(x, 0), this.width - 1);
        y = Math.min(Math.max(y, 0), this.height - 1);

        let w = this.width
        let h = this.height
        if ((x >= 0) && (x < w) && (y >= 0) && (y < h)) {
            // Pixel is in-bounds. Rotate coordinates if needed.
            switch (this.rotation) {
                case 1:
                    gfx_swap(x, y);
                    x = w - x - 1;
                    break;
                case 2:
                    x = w - x - 1;
                    y = h - y - 1;
                    break;
                case 3:
                    gfx_swap(x, y);
                    y = h - y - 1;
                    break;
            }

            let index = Math.floor((y * w + x) / 8);

            switch (color) {

                case COLOR.WHITE:
                    
                    this.buffer[index] |= (1 << (x & 7));
                    //buffer[y * w / 8 + Math.floor(x / 8)] |= (1 << (x & 7));
                    //buffer[x + (y / 8) * w] |= (1 << (y & 7));
                    break;
                case COLOR.BLACK:
                    this.buffer[index] &= ~(1 << (x & 7));
                    //buffer[x + (y / 8) * w] &= ~(1 << (y & 7));
                    break;
                case COLOR.INVERSE:
                    this.buffer[index] ^= (1 << (x & 7));
                    //buffer[x + (y / 8) * w] ^= (1 << (y & 7));
                    break;

            }
        }
    }

    getPixel(x, y) {

        
        x = Math.floor(x + 0.5);
        y = Math.floor(y + 0.5);

        let w = this.width
        let h = this.height
        if ((x >= 0) && (x < w) && (y >= 0) && (y < h)) {
            // Pixel is in-bounds. Rotate coordinates if needed.
            switch (this.rotation) {
                case 1:
                    gfx_swap(x, y);
                    x = w - x - 1;
                    break;
                case 2:
                    x = w - x - 1;
                    y = h - y - 1;
                    break;
                case 3:
                    gfx_swap(x, y);
                    y = h - y - 1;
                    break;
            }
            let index = Math.floor((y * w + x) / 8);
            //return buffer[y][Math.floor(x / 8)] & (1 << (x & 7));
            let shift = (1 << (x & 7));
            let val = this.buffer[index];
            let res = val & shift; 
            return res;
        }
        return false; // Pixel out of bounds
    }

    getBufferByteIndex(x, y) {

        x = Math.floor(x + 0.5);
        y = Math.floor(y + 0.5);

        let w = this.width
        let h = this.height
        if ((x >= 0) && (x < w) && (y >= 0) && (y < h)) {
            // Pixel is in-bounds. Rotate coordinates if needed.
            switch (this.rotation) {
                case 1:
                    gfx_swap(x, y);
                    x = w - x - 1;
                    break;
                case 2:
                    x = w - x - 1;
                    y = h - y - 1;
                    break;
                case 3:
                    gfx_swap(x, y);
                    y = h - y - 1;
                    break;
            }
            //return buffer[y][Math.floor(x / 8)] & (1 << (x & 7));
            return Math.floor((y * w + x) / 8);
        }
        return false; // Pixel out of bounds

    }

    drawLine(x0, y0, x1, y1, color) {
        var steep = Math.abs(y1 - y0) > Math.abs(x1 - x0);
        if (steep) {
            let temp = x0;
            x0 = y0;
            y0 = temp;

            temp = x1;
            x1 = y1;
            y1 = temp;
        }

        if (x0 > x1) {

            let temp = x0;
            x0 = x1;
            x1 = temp;

            temp = y0;
            y0 = y1;
            y1 = temp;
        }

        var dx, dy;
        dx = x1 - x0;
        dy = Math.abs(y1 - y0);

        var err = dx / 2;
        var ystep;

        if (y0 < y1) {
            ystep = 1;
        } else {
            ystep = -1;
        }

        for (; x0 <= x1; x0++) {
            if (steep) {
                this.drawPixel(y0, x0, color);
            } else {
                this.drawPixel(x0, y0, color);
            }
            err -= dy;
            if (err < 0) {
                y0 += ystep;
                err += dx;
            }
        }
    }
}



// /*!
//     @brief  Return the index of the byte containing the given pixel in display buffer.
//     @param  x
//             Column of display -- 0 at left to (screen width - 1) at right.
//     @param  y
//             Row of display -- 0 at top to (screen height -1) at bottom.
//     @return the byte in buffer
//     @note   Reads from buffer contents; may not reflect current contents of
//             screen if display() has not been called.
// */
// uint16_t GFX_Buffer::getBufferByteIndex(int16_t x, int16_t y) {
//     if ((x >= 0) && (x < width()) && (y >= 0) && (y < height()))
//     {
//         // Pixel is in-bounds. Rotate coordinates if needed.
//         switch (getRotation())
//         {
//         case 1:
//             gfx_swap(x, y);
//             x = WIDTH - x - 1;
//             break;
//         case 2:
//             x = WIDTH - x - 1;
//             y = HEIGHT - y - 1;
//             break;
//         case 3:
//             gfx_swap(x, y);
//             y = HEIGHT - y - 1;
//             break;
//         }

// #ifdef ROSE_SIMULATOR
//         return y * ((WIDTH + 7) / 8) + (x / 8);
// #else
//         return x + (y / 8) * WIDTH;
// #endif

//         // return (buffer[x + (y / 8) * WIDTH] & (1 << (y & 7)));
//     }
//     return 0; // Pixel out of bounds
// }
