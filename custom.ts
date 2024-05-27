




/**
 * Custom blocks
 */
//% weight=100 color=#0fbc11 icon="\uf247"
namespace tegneRobot {

    /**
    * Created by BG8 2024 UiT
    * Henrik Mauroy, hmauroy@gmail.com
    * Oliver Sokol
    * Mads Hansen
    * Use this file to define custom functions and blocks.
    * Read more at https://makecode.microbit.org/blocks/custom
    */

    let pca_register = 0;


    const machine = {
        currentPosition: { x: 0, y: 0 },
        direction: { x: 1, y: 1 },
    };

    const pinStates = {
        //% stepper pins: pin13=stepperX, pin15=stepperY, pin14=dirX, pin16=dirY
        stepperX: 0,
        dirX: 0,
        stepperY: 0,
        dirY: 0
    };



    export const draw = {
        pulseInterval: 400,
        penDown: false,
        isDrawing: true,
        targetPoint: { x: 0, y: 0 },
        previousTime: 0,
        pulseHigh: true,
        nextXStep: 0,
        nextYStep: 0,
    };


    export const bresenham = {
        err: 0,
        err2: 0,
        dx: 0,
        dy: 0,
        sx: 0,
        sy: 0
    };

    enum penLifted {
        //% block="lift"
        true,
        //% block="pendown"
        false,
    }

    //% help=HomeHead/draw weight=77
    //% block="Home Head"  icon="\uf204" blockGap=8
    export function homeHead() {
        /*
         * Moves head to 0,0 (upper left corner).
         * Moves steppers too much into negative direction.
         */
        liftPen();
        moveHeadTo(0, 0);
        //ledOff();    // sends number 32 to i2c slave.
        servos.P0.stop();
    }

    /**
    * Move head to xoordinate in absolute millimeters
    * @param xPosition - Coordinate on X axis in millimeters. Float is accepted as input.
    * @param yPosition - Coordinate on y axis in millimeters.
    */
    //%help = moveHeadTo / draw weight = 77
    //% block="Move Head To|xCoordinate %xPosition|yCoordinate %yPosition" blockGap=8
    //% xPosition.min=0 yPosition.min=0
    //% xPosition.fieldOptions.precision=1 yPosition.fieldOptions.precision=1 
    export function moveHeadToMM(xPosition: number, yPosition: number) {
        const stepsPerMM = Math.ceil(5000 / 62.0);
        let x = xPosition * stepsPerMM;
        let y = yPosition * stepsPerMM;
        moveHeadTo(x,y);
    }

    /**
    * Move head to xoordinate in pixel space
    * @param xPosition - Coordinate on X axis in pixel coordinates. Float is accepted as input.
    * @param yPosition - Coordinate on y axis in pixel coordinates
    */
    // The real moveHeadTo function.
    function moveHeadTo(xPosition: number, yPosition: number) {
        draw.targetPoint.x = Math.round(xPosition);
        draw.targetPoint.y = Math.round(yPosition);
        bresenham.dx = Math.abs(draw.targetPoint.x - machine.currentPosition.x);
        bresenham.dy = -Math.abs(draw.targetPoint.y - machine.currentPosition.y); // NB! Negative value!
        bresenham.err = bresenham.dx + bresenham.dy;
        //let realDx = draw.targetPoint.x - machine.currentPosition.x;
        //let realDy = draw.targetPoint.y - machine.currentPosition.y;
        //serialLog("target x,y: " + draw.targetPoint.x + "," + draw.targetPoint.y);
        //serialLog("" + draw.targetPoint.x + "," + draw.targetPoint.y);
        //serialLog("dx, dy: " + realDx + "," + realDy);

        if (machine.currentPosition.x < draw.targetPoint.x) {
            bresenham.sx = 1;
            //pins.digitalWritePin(DigitalPin.P14, 0);
            pinStates.dirX = 0;
        }
        else {
            bresenham.sx = -1;
            //pins.digitalWritePin(DigitalPin.P14, 1);
            pinStates.dirX = 1;
        }
        if (machine.currentPosition.y < draw.targetPoint.y) {
            bresenham.sy = 1;
            //pins.digitalWritePin(DigitalPin.P16, 1);
            pinStates.dirY = 1;
        }
        else {
            bresenham.sy = -1;
            //pins.digitalWritePin(DigitalPin.P16, 0);
            pinStates.dirY = 0;
        }

        //serialLog("dirx: " + bresenham.sx);
        //serialLog("diry: " + bresenham.sy);

        // Initializes the steps.
        draw.isDrawing = runBresenham(); // Updates global variables nextXStep, nextYStep to either +1, -1 or 0 for a step or not.


        while (draw.isDrawing) {
            
            if (draw.pulseHigh) {
                draw.previousTime = micros();
                //draw.previousTime = millis();
                draw.pulseHigh = !draw.pulseHigh; // Flips logic.
                //pins.digitalWritePin(DigitalPin.P13, 0);
                //pins.digitalWritePin(DigitalPin.P15, 0);
                pinStates.stepperX = 0;
                pinStates.stepperY = 0;
                stepSteppers();
            }
            else {
                draw.previousTime = micros();
                //draw.previousTime = millis();

                // Turns puls on or off. NB! Only 1 pulse/pixel.
                pinStates.stepperX = Math.abs(draw.nextXStep); // Absolute value because nextXStep can be +1/-1 or 0.
                pinStates.stepperY = Math.abs(draw.nextYStep);
                draw.pulseHigh = !draw.pulseHigh; // flips logic
                //pins.digitalWritePin(DigitalPin.P13, 1);
                //pins.digitalWritePin(DigitalPin.P15, 1);
                //serialLog("current x,y: " + machine.currentPosition.x + "," + machine.currentPosition.y);
                stepSteppers();

                // Calculate next step while we wait for next update.
                draw.isDrawing = runBresenham();
            }
            control.waitMicros(draw.pulseInterval);
            // Read different pins for inputs like A,B-buttons, accelerometer for emergency stop.
            // readDrawingBotSensors();
            

        } // END while (isDrawing)

        //serialLog("Finished move to.")
        //serialLog("current x,y: " + machine.currentPosition.x + "," + machine.currentPosition.y);
    }

    function runBresenham(): boolean {
        // Calculates steps in x and y directions. 0, -1 or 1
        // Updates global variables dx, dy, and error variable err.
        //serialLog("Bresenham current x,y: " + machine.currentPosition.x + "," + machine.currentPosition.y);
        //serialLog("Target x,y: " + draw.targetPoint.x + "," + draw.targetPoint.y);
        bresenham.err2 = 2 * bresenham.err;
        draw.nextXStep = 0;
        draw.nextYStep = 0;

        if (machine.currentPosition.x === draw.targetPoint.x && machine.currentPosition.y === draw.targetPoint.y) {
            return false;
        }
        if (bresenham.err2 >= bresenham.dy) {
            if (machine.currentPosition.x === draw.targetPoint.x) {
                return false;
            }
            // Update step and error
            bresenham.err = bresenham.err + bresenham.dy;
            machine.currentPosition.x = machine.currentPosition.x + bresenham.sx;
            draw.nextXStep = bresenham.sx;
        }
        if (bresenham.err2 <= bresenham.dx) {
            if (machine.currentPosition.y === draw.targetPoint.y) {
                return false;
            }
            bresenham.err = bresenham.err + bresenham.dx;
            machine.currentPosition.y = machine.currentPosition.y + bresenham.sy;
            draw.nextYStep = bresenham.sy;
        }
        return true;
    }





    /**
     * Logic calculates if x-stepper and y-stepper should run.
     * If so, each function is called individually to update the positions
     * of the drawing head by updating the "pinStates"-object.
     * For each pulse the information about pin states should be 
     * sent over I2C to the PCA9557 chip as one 8-bit number.

        Using function inside pins.cpp
        void digitalWritePin(DigitalPin name, int value) {
            PINOP(setDigitalValue(value));
        }
        HIGH = Mot Klokka
        LOW = Med Klokka
     */
    //% help=stepSteppers/draw weight=77
    //% block="Step steppers"  icon="\uf204" blockGap=8
    export function stepSteppers() {
        //serialLog("stepperX: " + pinStates.stepperX + ", stepperY: " + pinStates.stepperY);
        // Read from pinStates object and write using digitalWrite()
        pins.digitalWritePin(DigitalPin.P13, pinStates.stepperX);
        pins.digitalWritePin(DigitalPin.P14, pinStates.dirX);
        pins.digitalWritePin(DigitalPin.P15, pinStates.stepperY);
        pins.digitalWritePin(DigitalPin.P16, pinStates.dirY);

    }


    export function serialLog(text: string) {
        serial.writeLine(text);
    }

    //% blockId="setI2CPins" block="set i2c data to %sdaPin and clock to %sclPin|"
    //% shim=i2crr::setI2CPins
    //% sdaPin.defl=DigitalPin.P1 sclPin.defl=DigitalPin.P2
    //% group="micro:bit (V2)"
    export function setI2CPins(sdaPin: DigitalPin, sclPin: DigitalPin): void {
        // Per https://github.com/microsoft/pxt-microbit/issues/4292
        0;
    }

    /*
    * ~1.0 millisecond pulse all right
    * ~1.5 millisecond pulse center
    * ~2.0 millisecond pulse all left
    */
    //% block="Lift pen"  icon="\uf204" blockGap=8
    export function liftPen(): void {
        //% Lifts the pen by moving the servo "upwards"
        servos.P0.setAngle(75);
        basic.pause(300);
    }

    //% block="Lower pen"  icon="\uf204" blockGap=8
    export function lowerPen(): void {
        //% Lowers the pen by moving the servo past middle position.
        //serialLog("Pen lowered.")
        servos.P0.setAngle(100);
        basic.pause(200);
    }

    export function increaseSpeed() {
        draw.pulseInterval -= 100;
        if (draw.pulseInterval <= 200) {
            draw.pulseInterval = 200;
        }
    }

    export function decreaseSpeed() {
        draw.pulseInterval += 100;
        if (draw.pulseInterval >= 1000) {
            draw.pulseInterval = 1000;
        }
    }
  
    function micros() {
        // Get time ellapsed since app start in microseconds.
        return input.runningTimeMicros();
    }

    function millis() {
        // Get time ellapsed since app start in milliseconds.
        return input.runningTime();
    }

    //% block="Start drawings: Move head to upper left, push button B"  icon="\uf204" blockGap=8
    export function startDrawing() {
        let isWaiting = true;
        let startEvent = 1;
        let startEventValue = 1;
        let lastTime = input.runningTime();
        let displayOn = true;
        servos.P0.stop();
        serial.writeLine("Initiated drawing robot!");
        // Sets button B to HIGH
        pins.digitalWritePin(DigitalPin.P11, 1);
        // Initialize PCA9557
        i2crr.setI2CPins(DigitalPin.P1, DigitalPin.P2)
        ledOff();    // sends number 32 to i2c slave to turn off servo
        basic.showLeds(`
        . . # . .
        . . . # .
        # # # # #
        . . . # .
        . . # . .
        `);
        control.runInParallel(function () {
            while (isWaiting) {
                control.waitMicros(10000);
                // Detects press of button B when it is pulled LOW
                if (pins.digitalReadPin(DigitalPin.P11) === 0) {
                    // Turn OFF B-button setting it HIGH
                    pins.digitalWritePin(DigitalPin.P11, 1);
                    isWaiting = false;
                    // TODO: Enable pin turns on for stepper-drivers.

                    
                    control.raiseEvent(startEvent, startEventValue);
                }
                // Blink the display
                if (millis() - lastTime >= 1000) {
                    lastTime = input.runningTime();
                    if (displayOn) {
                        basic.clearScreen();
                    }
                    else {
                       basic.showLeds(`
                    . . # . .
                    . . . # .
                    # # # # #
                    . . . # .
                    . . # . .
                    `); 
                    }
                    displayOn = !displayOn;
                }
            }
        })
        // Here we halt the program by waiting for event.
        control.waitForEvent(startEvent, startEventValue);
        showOkIcon();
        liftPen();
        basic.pause(500);
        showStatusIcon();
    }

    //% block="Show OK icon" icon="\uf204" blockGap=8
    export function showOkIcon() {
        basic.showIcon(IconNames.Yes, 500);
    }

    /*
    * Lights up the lower row of leds as a bar graph related to
    * the amount of drawing left for the current object being drawn.
    * How to implement this is not straight forward.
    */
    //% block="Show status icon" icon="\uf204" blockGap=8
    export function showStatusIcon() {
        basic.clearScreen();
        // Show only a status led
        led.plot(0, 4);
    }

    //% block="End drawing and home head" icon="\uf204" blockGap=8
    export function endDrawing() {
        basic.showString("Done!");
        showOkIcon();
        homeHead();
    }

    function readDrawingBotSensors() {
        // TODO: Read different pins for inputs like A,B-buttons, accelerometer for emergency stop.
    }


    let PCA9557_ADDR = 24
    let CONFIGURATION_MODE = 3
    let PIN_CONFIGURATION = 219
    let OUTPUT_REGISTER = 1
    let INPUT_REGISTER = 0
    let pca_buffer = pins.createBuffer(2);


    // Set registers: first write config byte, then sequential write PIN CONFIG.
    //% block
    export function setreg(): void {
        pca_buffer[0] = 3;
        pca_buffer[1] = PIN_CONFIGURATION;
        pins.i2cWriteBuffer(PCA9557_ADDR, pca_buffer);
    }

    // Read register. Write config byte, then read register
    //% block
    export function getreg(): number {
        pins.i2cWriteNumber(PCA9557_ADDR, INPUT_REGISTER, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(PCA9557_ADDR, NumberFormat.UInt8BE);
    }

    /*
    * Request data from Arduino    
    */
    //% block "Request Data from Arduino"
    function requestData(): string {
        // Request 32 bytes from the slave device
        pins.i2cWriteNumber(PCA9557_ADDR, 0, NumberFormat.UInt8LE, true)
        let receivedData = i2cReadString(PCA9557_ADDR, 32, NumberFormat.UInt8LE)

        //serialLog(receivedData);
        return receivedData;
    }


    /*
    * Internal function to construct a string from I2C-communication.
    */
    function i2cReadString(address: number, maxLength: number, delimiter: number = 0): string {
        let receivedData = ""
        let buf = pins.i2cReadBuffer(address, maxLength, false)

        for (let i = 0; i < maxLength; i++) {
            let charCode = buf.getNumber(NumberFormat.UInt8LE, i)
            //serialLog("" + charCode);
            if (charCode == delimiter || charCode == 0) {
                break
            }
            receivedData += String.fromCharCode(charCode)
        }

        return receivedData
    }

    // Write to register. Write config byte, then sequential write PIN CONFIG. IO5 LOW IO2 HIGH
    //% block
    export function ledON(): void {
        pca_buffer[0] = OUTPUT_REGISTER;
        pca_buffer[1] = 4;
        pins.i2cWriteBuffer(PCA9557_ADDR, pca_buffer);
    }

    // Read register. Write config byte, then sequential write PIN CONFIG. IO5 HIGH IO2 LOW
    //% block
    export function ledOff(): void {
        pca_buffer[0] = OUTPUT_REGISTER;
        pca_buffer[1] = 32;
        pins.i2cWriteBuffer(PCA9557_ADDR, pca_buffer);
    }


    // Figures definitions
    /**
    * Draws a square
    * @param xPosition - Upper left coordinate on X axis
    * @param yPosition - Upper left coordinate on y axis
    * @param lengthOfSide - length of size in mm. 5000 steps = 62.0 mm Y-axis, 64.6 mm X-axis approximately. Must be set by user after running calibrationX() and calibrationY() blocks once. # TODO: Not implemented yet.
    * @param rotation - rotation of square calculated by rotating around center point calculated by averaging all 4 corners. #TODO: not implemented yet. 
    */
    //% block="Square|upper left Xpos %xPosition|upper left Ypos %yPosition| length of side %lengthOfSide| rotation %rotation |penLifted %lift" blockGap=8
    //% xPosition.min=0 yPosition.min=0 lengthOfSide.defl=10 lift.defl=true
    export function square(xPosition: number, yPosition: number, lengthOfSide: number, rotation: number = 0, lift = true): void {
        const stepsPerMM = Math.ceil(5000 / 62.0);
        const origin = { x: xPosition * stepsPerMM, y: yPosition * stepsPerMM };
        const size = lengthOfSide * stepsPerMM;
        moveHeadTo(origin.x, origin.y);
        lowerPen();
        moveHeadTo(origin.x + size, origin.y);
        moveHeadTo(origin.x + size, origin.y + size);
        moveHeadTo(origin.x, origin.y + size);
        moveHeadTo(origin.x, origin.y);
        if (lift) {
            liftPen();
        }
        serialLog("Finished square");

    }

    /* Draws a rectangle with sides a and b
    * @param xPosition - Upper left coordinate on X axis
    * @param yPosition - Upper left coordinate on y axis
    * @param length - length of size in mm. 5000 steps = 62.0 mm Y-axis, 64.6 mm X-axis approximately. Must be set by user after running calibrationX() and calibrationY() blocks once. # TODO: Not implemented yet.
    * @param height - length of size in mm. 5000 steps = 62.0 mm Y-axis, 64.6 mm X-axis approximately. Must be set by user after running calibrationX() and calibrationY() blocks once. # TODO: Not implemented yet.
    * @param rotation - rotation of square calculated by rotating around center point calculated by averaging all 4 corners. #TODO: not implemented yet. 
    */
    //% block="Rectangle|upper left Xpos %xPosition|upper left Ypos %yPosition| length %length| height %height| rotation %rotation |penLifted %lift" blockGap=8
    //% xPosition.min=0 yPosition.min=0 length.defl=20 height.defl=10 lift.defl=true
    export function rectangle(xPosition: number, yPosition: number, length: number, height: number, rotation: number = 0, lift = true): void {
        const stepsPerMM = Math.ceil(5000 / 62.0);
        const origin = { x: xPosition * stepsPerMM, y: yPosition * stepsPerMM };
        const l = length * stepsPerMM;
        const h = height * stepsPerMM;
        moveHeadTo(origin.x, origin.y);
        lowerPen();
        moveHeadTo(origin.x + l, origin.y);
        moveHeadTo(origin.x + l, origin.y + h);
        moveHeadTo(origin.x, origin.y + h);
        moveHeadTo(origin.x, origin.y);
        if (lift) {
            liftPen();
        }
        //serialLog("Finished rectangle");

    }


    /**
     * Draws a circle
     * @param x coordinate
     * @param y coordinate
     * @param r radius
     * @param lift if pen should lift after drawing finished
     */
    //% help=circle/draw weight=77
    //% block="Circle|centerX %x|centerY %y|radius %r|penLifted %lift" icon="\uf1db" blockGap=8
    //% x.min=0 x.max=120 y.min=0 y.max=100 r.min=3 r.max=50
    //% x.fieldOptions.precision=1 y.fieldOptions.precision=1 r.defl=3 lift.defl=true
    export function circle(centerX: number, centerY: number, r: number, lift = true): void {
        const stepsPerMM = Math.ceil(5000 / 62.0);
        const origin = { x: centerX * stepsPerMM, y: centerY * stepsPerMM };
        const segment_length = 2;
        const n_segments = (2 * Math.PI * r) / segment_length;
        const n = Math.ceil(n_segments);
        const d_theta = 2 * Math.PI / n;
        let theta = 0;

        const radius = Math.ceil(r * stepsPerMM);
        let x0 = origin.x + radius;
        let y0 = origin.y
        moveHeadTo(x0, y0);
        lowerPen();

        // Run for loop of n line segments.
        for (let i = 0; i < n; i++) {
            theta = theta + d_theta;
            draw.targetPoint.x = radius * Math.cos(theta) + origin.x;
            draw.targetPoint.y = radius * Math.sin(theta) + origin.y;
            moveHeadTo(draw.targetPoint.x, draw.targetPoint.y);
        }

        if (lift) {
            liftPen();
        }
        serialLog("Finished circle");

    }

    /**
    * Draws a triangle
    * @param x1 - Coordinate on X axis in mm
    * @param y1 - Coordinate on y axis
    * @param x2 - Coordinate on X axis
    * @param y2 - Coordinate on y axis
    * @param x3 - Coordinate on X axis
    * @param y3 - Coordinate on y axis
    * @param rotation - rotation of triangle calculated by rotating around center point calculated by averaging all 3 corners. #TODO: not implemented yet. 
    */
    //% block="Triangle|x1 %x1|y1 %y1|x2 %x2|y2 %y2|x3 %x3|y3 %y3|rotation %rotation |penLifted %lift" blockGap=8
    //% x1.min=0, x2.min=0, x3.min=0, y1.min=0 y2.min=0 y3.min=0 lift.defl=true
    export function triangle(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, rotation: number = 0, lift = true): void {
        showStatusIcon();
        const stepsPerMM = Math.ceil(5000 / 62.0);
        const positions = { 
            x1: x1 * stepsPerMM, 
            y1: y1 * stepsPerMM,
            x2: x2 * stepsPerMM,
            y2: y2 * stepsPerMM,
            x3: x3 * stepsPerMM,
            y3: y3 * stepsPerMM
            };
        moveHeadTo(positions.x1, positions.y1);
        lowerPen();
        moveHeadTo(positions.x2, positions.y2);
        moveHeadTo(positions.x3, positions.y3);
        moveHeadTo(positions.x1, positions.y1);
        if (lift) {
            liftPen();
        }
        serialLog("Finished triangle");

    }

    // Define a type for the elements that can be either a string or a number.
    type SvgElement = any | number;

    // Define the interface for the array where each inner array can contain any number of SvgElement.
    type SvgArray = SvgElement[][];

    function pythagoras(dx: SvgElement, dy: SvgElement): number {
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    /**
    * Draws the SVG
    * @param svgString - Bezier curves on this format: "[[\"M\",6.962,0.115,\"C\",10.833,0.138,17.167,0.138,21.038,0.115,\"C\",24.91,0.092,21.742,0.074,14,0.074,\"C\",6.258,0.074,3.09,0.092,6.962,0.115]]"
    * Webpage exports JSON with double quotes which are escaped automatically by MakeCode editor.
    */
    //% block="SVGArr|SVG Array %svgArr |penLifted %lift" blockGap=8
    export function svg2(svgArr: SvgArray, lift = true): void {
        serialLog("Draws SVG");
        /*
        svgArr.forEach(arr => {
            arr.forEach(val => {
                serialLog("" + val);
            });
        });
        */
        const stepsPerMM = 5000 / 62.0;
        // Run through array
        let lastCoordinates: SvgElement[] = [];
        let coordinates: SvgElement[] = [];
        let n_segments = 1;
        let curveLength: number = 0;
        let x0: number, y0: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number;
        for (let i = 0; i < svgArr.length; i++) {
            for (let j = 0; j < svgArr[i].length; j++) {
                if (svgArr[i][j] === "M") {
                    // Absolute move
                    serialLog("M " + svgArr[i][j + 1] + "," + svgArr[i][j+2]);

                    let x = svgArr[i][j + 1] * stepsPerMM;
                    let y = svgArr[i][j + 2] * stepsPerMM;
                    liftPen();
                    moveHeadTo(x,y);
                    lowerPen();
                    lastCoordinates = [svgArr[i][j + 1], svgArr[i][j + 2]];
                    coordinates = [];
                    j += 2;

                }
                if (svgArr[i][j] === "C") {
                    // Cubic bezier
                    let drawCoords: (number | number)[][] = [];
                    coordinates = []; // Initialize empty array to store lastCoordinates and the next 6 control points.
                    coordinates = coordinates.concat(lastCoordinates);
                    coordinates = coordinates.concat([svgArr[i][j + 1], svgArr[i][j + 2], svgArr[i][j + 3], svgArr[i][j + 4], svgArr[i][j + 5], svgArr[i][j + 6]]);
                    serialLog("C " + svgArr[i][j + 1] + "," + svgArr[i][j + 2]);
                    //serialLog("C");
                    /*
                    coordinates.forEach(coord => {
                        serial.writeString("" + coord + "," );
                    });
                    serial.writeLine("");
                    */
                    lastCoordinates = [svgArr[i][j + 5], svgArr[i][j + 6]];
                    // Calculate approximate length of segment and divide bezier curve into 2mm long segments.
                    curveLength = pythagoras(coordinates[6] - coordinates[0], coordinates[7] - coordinates[1]);
                    //serialLog("" + curveLength);

                    n_segments = Math.ceil(curveLength / 2);
                    //n_segments = 30;
                    //serialLog("n_segments: " + n_segments);
                    x0 = coordinates[0];
                    y0 = coordinates[1];
                    x1 = coordinates[2];
                    y1 = coordinates[3];
                    x2 = coordinates[4];
                    y2 = coordinates[5];
                    x3 = coordinates[6];
                    y3 = coordinates[7];
                    // Calculate each point along bezier curve.
                    // http://rosettacode.org/wiki/Cubic_bezier_curves#C
                    for (let k = 0; k < n_segments; k++) {
                        let t = k / n_segments;
                        let a = Math.pow((1.0 - t), 3);
                        let b = 3.0 * t * Math.pow((1.0 - t), 2);
                        let c = 3.0 * Math.pow(t, 2) * (1.0 - t);
                        let d = Math.pow(t, 3);

                        let x = a * x0 + b * x1 + c * x2 + d * x3;
                        let y = a * y0 + b * y1 + c * y2 + d * y3;

                        x = x * stepsPerMM;
                        y = y * stepsPerMM;

                        //serialLog("" + x + "," + y);
                        //drawCoords.push([x, y]);
                        moveHeadTo(x,y);
                    }

                    // Draw the points
                    /*
                    drawCoords.forEach(point => {
                        //serialLog("" + point[0] + "," + point[1]);
                        moveHeadTo(point[0], point[1]);
                    })
                    */
                    j += 6;

                }
                if (svgArr[i][j] === "L") {
                    // Line
                    coordinates = [svgArr[i][j + 1], svgArr[i][j + 2], svgArr[i][j + 3], svgArr[i][j + 4]]; // Start and end coordinates are indicated by L segment.
                    
                    serialLog("L");
                    coordinates.forEach(coord => {
                        serial.writeString("" + coord + ",");
                    });
                    serial.writeLine("");
                    // Move to start of L(ine)
                    //liftPen();
                    let x = svgArr[i][j + 1] * stepsPerMM;
                    let y = svgArr[i][j + 2] * stepsPerMM;
                    moveHeadTo(x,y);
                    // Draw the L(ine)
                    //lowerPen();
                    x = svgArr[i][j + 1] * stepsPerMM;
                    y = svgArr[i][j + 2] * stepsPerMM;
                    moveHeadTo(x, y);
                    
                    lastCoordinates = [svgArr[i][j + 3], svgArr[i][j + 4]];
                    j += 4;

                }
            }

        }

        if (lift) {
            liftPen();
        }
        serialLog("Finished SVG drawing");
        serialLog("current pos: " + machine.currentPosition.x + "," + machine.currentPosition.y);

        basic.pause(500);
    }

    /**
    * Draws the SVG from a JSON-string
    * @param svgString - Bezier curves on this format: "[[\"M\",6.962,0.115,\"C\",10.833,0.138,17.167,0.138,21.038,0.115,\"C\",24.91,0.092,21.742,0.074,14,0.074,\"C\",6.258,0.074,3.09,0.092,6.962,0.115]]"
    * Webpage exports JSON with double quotes which are escaped automatically by MakeCode editor.
    */
    //% block="SVG|SVG string %svgString |penLifted %lift" blockGap=8
    export function svg(svgString: string, lift = true): void {
        const stepsPerMM = 5000 / 62.0;
        let svgArr: (any | (string | number))[][] = JSON.parse(svgString);
        // Run through array
        let lastCoordinates: number[] = [];
        let coordinates: number[] = [];
        let n_segments = 1;
        let curveLength = 0;
        let x0: number, y0: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number;
        for (let i=0; i<svgArr.length; i++) {
            for (let j=0; j<svgArr[i].length; j++) {
                if (svgArr[i][j] === "M") {
                    // Absolute move
                    //serialLog("M " + svgArr[i][j + 1] + "," + svgArr[i][j+2]);
                    lastCoordinates = [svgArr[i][j + 1], svgArr[i][j + 2]];
                    coordinates = [];
                    let x = svgArr[i][j + 1] * stepsPerMM;
                    let y = svgArr[i][j + 2] * stepsPerMM;
                    liftPen();
                    moveHeadTo(x, y);
                    lowerPen();
                    j += 2;

                }
                if (svgArr[i][j] === "C") {
                    // Cubic bezier
                    let drawCoords: (number|number)[][] = [];
                    coordinates = []; // Initialize empty array to store lastCoordinates and the next 6 control points.
                    coordinates = coordinates.concat(lastCoordinates);
                    coordinates = coordinates.concat( [ svgArr[i][j + 1], svgArr[i][j + 2], svgArr[i][j + 3], svgArr[i][j + 4], svgArr[i][j + 5], svgArr[i][j + 6] ] );
                    //serialLog("C " + svgArr[i][j + 1] + "," + svgArr[i][j + 2]);
                    //serialLog("C");
                    /*
                    coordinates.forEach(coord => {
                        serial.writeString("" + coord + "," );
                    });
                    serial.writeLine("");
                    */
                    lastCoordinates = [svgArr[i][j + 5], svgArr[i][j + 6]];
                    // Calculate approximate length of segment and divide bezier curve into 2mm long segments.
                    curveLength = pythagoras(coordinates[6]-coordinates[0], coordinates[7]-coordinates[1]);

                    n_segments = Math.ceil(curveLength / 2);
                    //n_segments = 30;
                    //serialLog("n_segments: " + n_segments);
                    x0 = coordinates[0];
                    y0 = coordinates[1];
                    x1 = coordinates[2];
                    y1 = coordinates[3];
                    x2 = coordinates[4];
                    y2 = coordinates[5];
                    x3 = coordinates[6];
                    y3 = coordinates[7];
                    // Calculate each point along bezier curve.
                    // http://rosettacode.org/wiki/Cubic_bezier_curves#C
                    for (let k = 0; k < n_segments; k++) {
                        let t = k / n_segments;
                        let a = Math.pow((1.0 - t), 3);
                        let b = 3.0 * t * Math.pow((1.0 - t), 2);
                        let c = 3.0 * Math.pow(t, 2) * (1.0 - t);
                        let d = Math.pow(t, 3);

                        let x = a * x0 + b * x1 + c * x2 + d * x3;
                        let y = a * y0 + b * y1 + c * y2 + d * y3;

                        x = x * stepsPerMM;
                        y = y * stepsPerMM;

                        //serialLog("" + x + "," + y);
                        //drawCoords.push([x, y]);
                        moveHeadTo(x, y);

                        serialLog("" + x + "," + y);
                        //drawCoords.push([x * stepsPerMM, y * stepsPerMM]);
                    }

                    /*
                    // Draw the points
                    drawCoords.forEach(point => {
                        //serialLog("" + point[0] + "," + point[1]);
                        moveHeadTo(point[0], point[1]);
                    })
                    */

                    j += 6;

                }
                if (svgArr[i][j] === "L") {
                    // Line
                    coordinates = [svgArr[i][j + 1], svgArr[i][j + 2], svgArr[i][j + 3], svgArr[i][j + 4] ] ; // Start and end coordinates are indicated by L segment.
                    /*
                    serialLog("L");
                    coordinates.forEach(coord => {
                        serial.writeString("" + coord + ",");
                    });
                    serial.writeLine("");
                    */
                    lastCoordinates = [svgArr[i][j + 3], svgArr[i][j + 4]];
                    j += 4;

                }
            }
            
        }

        if (lift) {
            liftPen();
        }
        serialLog("Finished SVG drawing");
        serialLog("current pos: " + machine.currentPosition.x + "," + machine.currentPosition.y);

        basic.pause(500);
    }

    /**
    * Draws the SVG from SD-card
    */
    //% block="SVG (SD-card) |penLifted %lift" blockGap=8
    //% lift.defl=true
    export function svgSdCard(lift = true): void {
        const stepsPerMM = 5000 / 62.0;
        let lastCoordinates: number[] = [];
        let coordinates: number[] = [];
        let n_segments = 1;
        let curveLength = 0;
        let x0: number, y0: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number;
        let isReadingSD = true;
        let line = "";
        while (isReadingSD) {
            line = requestData();           
            
            // Run through the line and extract commands and coordinates.
            // Split the data into parts by commas
            let parts = line.split(",");

            if (line.length === 0) {
                serialLog("Empty data returned");
                if (line === "EOF") {
                    if (lift) {
                        liftPen();
                    }
                    serialLog("Finished SVG drawing from SD-card");
                    serialLog("current pos: " + machine.currentPosition.x + "," + machine.currentPosition.y);

                    basic.pause(500);
                }
                continue;
            }

            // Check the first part to determine the type
            let type = parts[0];

            if (type === "M") {
                // Handle 'M' type data
                let x = parseFloat(parts[1]);
                let y = parseFloat(parts[2]);

                serialLog("M " + x + "," + y);

                x = x * stepsPerMM;
                y = y * stepsPerMM;
                liftPen();
                moveHeadTo(x, y);
                lowerPen();


            } 
            else if (type === "C") {
                // Handle 'C' type data
                x0 = parseFloat(parts[1]) * stepsPerMM;
                y0 = parseFloat(parts[2]) * stepsPerMM;
                x2 = parseFloat(parts[3]) * stepsPerMM;
                y2 = parseFloat(parts[4]) * stepsPerMM;
                x3 = parseFloat(parts[5]) * stepsPerMM;
                y3 = parseFloat(parts[6]) * stepsPerMM;

                serialLog("C " + x3 + "," + y3);
                // Simple move to end point
                moveHeadTo(x3, y3);

            }
            else if (type === "L") {
                // Handle 'L' type data
                serialLog("L");
            }
            else {
                // Handle unexpected type
                serialLog("Error");
            }

            //basic.pause(500);            
        }

    }










}

