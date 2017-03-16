/*
 Linear_Array_Sensor_Subpixel_Visualizer_V2.pde 
 Displays linear photodiode array pixel and shadow data with subpixel resolution
 
 Created by Douglas Mayhew,  March 16, 2017.
 
 MIT License
 
 Copyright (c) 2017 Douglas Mayhew
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 
 Notes:
 Set SENSOR_PIXELS to the correct value for the sensor used.
 Plots sensor or simulated data, convolves it to smooth it using an adjustable gaussian kernel, 
 plots the convolution output and the first differences of that output, and finds all shadows 
 or simulated shadows and reports their position with subpixel accuracy using quadratic interpolation.
 
 The shadow positions reported in the text display assume the first sensor pixel is pixel number 1.
 Shadows create one negative peak followed by one positive peak in the 1st difference plot.
 Simple mods are coming shortly for use in finding spectra peaks or laser peaks, etc, which are single peak,
 not double-peaked like the shadows, so need a slightly different peak finder. 
 
 Released into the public domain, except:
 * The function, 'makeGaussKernel1d' is made available as part of the book 
 * "Digital Image * Processing - An Algorithmic Introduction using Java" by Wilhelm Burger
 * and Mark J. Burge, Copyright (C) 2005-2008 Springer-Verlag Berlin, Heidelberg, New York.
 * Note that this code comes with absolutely no warranty of any kind.
 * See http://www.imagingbook.com for details and licensing conditions. 
 
 See: https://github.com/Mr-Mayhem/DSP_Snippets_For_Processing
 
 * PanZoomController
 * @author Bohumir Zamecnik, modified by Doug Mayhew Dec 7, 2016
 * @license MIT
 * 
 * Inspired by "Pan And Zoom" by Dan Thompson, licensed under Creative Commons
 
 For more info on 3 point quadratic interpolation, see the subpixel edge finding method described in F. Devernay,
 A Non-Maxima Suppression Method for Edge Detection with Sub-Pixel Accuracy
 RR 2724, INRIA, nov. 1995
 http://dev.ipol.im/~morel/Dossier_MVA_2011_Cours_Transparents_Documents/2011_Cours1_Document1_1995-devernay--a-non-maxima-suppression-method-for-edge-detection-with-sub-pixel-accuracy.pdf
 
 The subpixel code has evolved quite a bit since I last first saw an example in a filiment width sensor:
 
 see Filament Width Sensor Prototype by flipper:
 https://www.thingiverse.com/thing:454584
 
 Serial encoding and decoding protocol from user Robin2 in forum.arduino.cc:
 https://forum.arduino.cc/index.php?topic=225329.0
 
 This sketch is able to run the subpixel position code against various data sources. 
 The sketch can synthesize test data like square impulses, to verify that the output is 
 reporting what it should and outputs are phased correctly relative to each other, 
 but this sketch is mainly concerned with displaying and measuring 
 shadow positions in live sensor serial data from a TSL1402R or TSL1410R linear photodiode array,
 or from a video camera line-grab across the middle of the video frame. 
 
 To feed this sketch with data from the TSL1402R or TSL1410R or similiar AMS or IC Haus sensors, 
 see my 2 related projects at: https://github.com/Mr-Mayhem/
 Teensy36_Read_Linear_Array_Send_Binary_Serial_ILI9341
 and...
 Ardino_Read_Linear_Array_Send_Binary_Serial
 
 This is a work in progress, but the subpixel code works nicely, and keeps evolving.
 I tried to keep it fast as possible. If you want higher speed, turn off unnecessary graphics. 
 Also, the TSL1402R, being 1/5 the size of the TSL1410R, 
 is 5 times faster. So if you really don't need the 1280 pixel width of the TSL1410R, use the
 256 pixel TSL1402R or one of the smaller 128 pixel jobs. 
 There are others in-between and I haven't played with them yet, but you can set the pixel count and go.
 IC Haus Germany makes compatable sensors as well.
 
 If you find any bugs, let me know via github or the Teensy forums in the following thread:
 https://forum.pjrc.com/threads/39376-New-library-and-example-Read-TSL1410R-Optical-Sensor-using-Teensy-3-x
 
 We still have some more refactoring and features yet to apply. I want to add:
 windowing and thresholding to reduce the workload of processing all data to processing only some data
 interpolation is not yet in this one.
 
 My update goals:
 1. Option to send shadow position and width instead of raw data, ***(Done!)***
 2. Send a windowed section containing only the interesting data, rather than all the data.
 3. Auto-Calibration using drill bits, dowel pins, etc.
 4. Multiple angles of led lights shining on the target, so multiple exposures may be compared 
 for additional subpixel accuracy, or a faster solution - multiple slits casting shadows 
 and averaging the shadow subpixel positions.
 5. Add data window zoom and scrolling ***(Done!)***
 6. Add measurement waterfall history display  ***(Done!)***
 7. Bringing the core of the position and subpixel code into Arduino for Teensy 3.6 ***(Done!)***
 8. Option to averaging two or more frames or sub-frames (windowed processing)  ***(Done, but commented out for speed,
 Also commented out because Teensy 3.6 has ADC averaging, so probably a redundant capability)***
 9. Collect and display unlimited number of shadows ***(Done!)*** 
 
 // To run in simulated data mode, set signalSource to 1,2, or 4 or 6.
 // To run in sensor mode with the sister app on Teensy, set signalSource to 3, set Serial settings on this page
 // To run in video camera mode, set signalSource to 5, and plug in a USB webcam or USB microscope, etc.
 
 // To cast multiple shadows, place a comb over the sensor with a bright white LED light hanging from a desklamp above.
 // Insulated Arduino breadboard wires make a convenient narrow shadow caster
 // Have fun, post questions to the following pjrc.com forum: 
    https://forum.pjrc.com/threads/39376-New-library-and-example-Read-TSL1410R-Optical-Sensor-using-Teensy-3-x/
 */
// ==============================================================================================
// imports:

//import processing.serial.*;
import processing.video.*;

// ==============================================================================================
// Constants:

// unique byte used to sync the filling of byteArray to the incoming serial stream
final int PREFIX = 0xFF;
// ==============================================================================================
// Arrays:

volatile byte[] byteArray = new byte[0];          // array of encoded serial data bytes
volatile byte[] byteArrayDecoded = new byte[0];   // Decoded serial bytes
volatile  byte[] msgArrayDecoded = new byte[0];   // Arduino serial debug messages as bytes
volatile  byte[] varArrayDecoded = new byte[0];   // variables received from serial as bytes
volatile byte[] shadowArrayDecoded = new byte[0]; // shadow info width and position bytes received from serial

int[] sigGenOutput = new int[0];     // array for signal generator output
float[] kernel = new float[0];       // array for impulse response, or kernel
int videoArray[] = new int[0];       // holds one row of video data, a row of integer pixels copied from the video image
float[] sineArray = new float[0];    // holds a one cycle sine wave, used to modulate Signal Generator output X axis
// ==============================================================================================
// Global Variables:

int HALF_SCREEN_HEIGHT;              // half the screen height, reduces division math work because it is used alot
int HALF_SCREEN_WIDTH;               // half the screen width, reduces division math work because it is used alot
int signalSource;                    // selects a signal data source
int kernelSource;                    // selects a kernel
int SENSOR_PIXELS;                   // real serial data, number of discrete data values, 1 per sensor pixel
int SENSOR_BIT_DEPTH;                // number of bits comprising each read sensor value as read by the ADC
int SIMULATED_SENSOR_PIXELS;         // simulated data, number of discrete data values, 1 per sensor pixel
int SIMULATED_SENSOR_BIT_DEPTH;      // number of bits comprising each simulated sensor value
int SIMULATED_SENSOR_WAVELENGTH;     // wavelength of simulated sensor data waveform
int N_BYTES_DATA;                    // we use 2 bytes to represent each sensor pixel
int N_BYTES_ENCODED_FRAME;           // N_BYTES_DECODED_FRAME x 2 
int KERNEL_LENGTH;                   // number of discrete values in the kernel array, set in setup() 
int KERNEL_LENGTH_MINUS1;            // kernel length minus 1, used to reduce math in loops
int HALF_KERNEL_LENGTH;              // Half the kernel length, used to correct convoltion phase shift
int availableBytesDraw;              // used to show the number of bytes present in the serial buffer
int gtextSize;                       // sizes all text, consumed by this page, dataplot class, legend class
int chartRedraws;                    // used to count sensor data frames
int bytesAvailable;                  // holds the number of bytes in the serial buffer, to tell if the reading is keeping up
float bitRate;                       // serial data bits per second, calculated in Serial_Thread (for displaying text)
float dataFrameRate;                 // serial data frames per second, calculated in Serial_Thread (for displaying text)
float shadowFrameRate;               // serial shadow info frames per second, calculated in Serial_Thread (for displating text)
boolean drawHasFinished = false;

float subpixelCenter = 0;
float subpixelWidth = 0;
float subpixelCenterLP = 0;
float subpixelWidthLP = 0;

// ==============================================================================================
// Set Objects

//Serial myPort;       // One Serial object, receives serial port data from Teensy 3.6 running sensor driver sketch
SerialThread serialReader;  // This object reads serial data on a seperate thread.
dataPlot DP1;        // One dataPlot object, handles plotting data with mouse sliding and zooming ability
SignalGenerator SG1; // Creates artificial signals for the system to process and display for testing & experientation
Capture video;       // create video capture object named video
// ==============================================================================================

void setup() {
  // Set the screen size to make it the size we want:
  //fullScreen();

  size(640, 640);

  // leave alone! Used in many places to center data at center of screen, width-wise
  HALF_SCREEN_WIDTH = width / 2;

  // leave alone! Used in many places to center data at center of screen, height-wise
  HALF_SCREEN_HEIGHT = height / 2;

  gtextSize = 10; // sizes all text, consumed by this page, dataplot class, legend class to space text y using this value plus padding
  // set framerate() a little above where increases don't speed it up much.
  // Also note, for highest speed, comment out drawing plots you don't care about.

  background(0);
  strokeWeight(1);
  textSize(gtextSize);
  println("SCREEN_WIDTH: " + width);
  println("SCREEN_HEIGHT: " + height);

  // ============================================================================================
  // 0 is default, dynamically created gaussian kernel
  kernelSource = 0; // <<< <<< Choose a kernel source (0 = dynamically created gaussian "bell curve"):

  // Create a kernelGenerator object, which creates a kernel and saves it's data into an array
  // 0: dynamically created gaussian kernel
  // 1: hard-coded gaussuan kernel (manually typed array values)
  // 2: laplacian of gaussian (LOG) kernel just to see what happens. Some laser subpixel papers like it, but experimental, not conventional;

  // ============================================================================================
  // You are encouraged to try different signal sources to feed the system

  signalSource = 3;  // <<< <<< Choose a signal source; 

  // 0: manually typed array data
  // 1: square impulse
  // 2: square wave 
  // 3: serial data from linear photodiode array sensor, use with sister Teensy 3.6 arduino sketch
  // 4: random perlin noise
  // 5: center height line grab from your video camera
  // 6: sine wave
  // 7: one cycle sine wave
  // =============================================================================================

  // Create a Signal Generator object, which generates a kernel and various test signals.

  SG1 = new SignalGenerator(this, 1.4);
  SG1.setKernelSource(kernelSource);
  SENSOR_PIXELS = 256; // set the number of pixels on the linear photodiode array sensor here. 
  SENSOR_BIT_DEPTH = 12;
  // Does not affect video or simulated data
  SIMULATED_SENSOR_PIXELS = 256;
  SIMULATED_SENSOR_BIT_DEPTH = 12;
  SIMULATED_SENSOR_WAVELENGTH = 32;

  N_BYTES_DATA = (SENSOR_PIXELS * 2);                  // 2 bytes represent each sensor pixel
  N_BYTES_ENCODED_FRAME = (N_BYTES_DATA * 2) + 4;      // Twice the number of data bytes, plus 4 signal bytes, 
  // times 2 for safe margin
  byteArray = new byte[N_BYTES_ENCODED_FRAME];         // init the array for received encoded data bytes
  byteArrayDecoded = new byte[N_BYTES_DATA];           // init the array for decoded serial data bytes
  msgArrayDecoded = new byte[2046]; 
  varArrayDecoded = new byte[8];
  shadowArrayDecoded = new byte[8];

  if (signalSource == 3) {
    sigGenOutput = SG1.signalGeneratorOutput(signalSource, SENSOR_PIXELS, SENSOR_BIT_DEPTH, SIMULATED_SENSOR_WAVELENGTH); // Serial data mode, data source, num of data points, height of peaks
  } else if (signalSource == 5) {
    sigGenOutput = SG1.signalGeneratorOutput(signalSource, SIMULATED_SENSOR_PIXELS, SIMULATED_SENSOR_BIT_DEPTH, SIMULATED_SENSOR_WAVELENGTH); // Video Mode, inputs are ignored
    SENSOR_PIXELS = videoArray.length;
  } else {
    sigGenOutput = SG1.signalGeneratorOutput(signalSource, SIMULATED_SENSOR_PIXELS, SIMULATED_SENSOR_BIT_DEPTH, SIMULATED_SENSOR_WAVELENGTH); // data source, num of data points, height of peaks
    SENSOR_PIXELS = sigGenOutput.length;
  }

  sineArray = SG1.oneCycleSineWaveFloats(256); // values used to move x to and fro as "modulation"

  // Create the dataPlot object, which handles plotting data with mouse sliding and zooming ability
  // dataStop set not past SENSOR_PIXELS, rather than SENSOR_PIXELS + KERNEL_LENGTH, to prevent convolution garbage at end 
  // from partial kernel immersion
  if (signalSource == 5) {
    // camera with 8 bit samples
    DP1 = new dataPlot(this, 0, 0, width, HALF_SCREEN_HEIGHT, SENSOR_PIXELS, SENSOR_BIT_DEPTH, gtextSize);
  } else { // assume 12 bit samples
    DP1 = new dataPlot(this, 0, 0, width, HALF_SCREEN_HEIGHT, SENSOR_PIXELS, SENSOR_BIT_DEPTH, gtextSize);
  }

  DP1.modulateX = true; // apply simulated shadow movement, half a pixel left and right in a sine wave motion

  // Serial Port Mode, Serial Settings =============================================

  if (signalSource == 3) {
    frameRate(500);
    noLoop();
    // Set up serial connection
    // Set to your Teensy COM port number to fix error, make sure it talks to Arduino software if stuck.

    // Linux Serial Port Notes:

    // Insure to follow the Teensy instructions on adding rules for serial to work in linux.
    // https://www.pjrc.com/teensy/td_download.html#linux_issues

    // Second, and take note because this bug bites at unexpected times, 
    // if this sketch connects error-free but freezes up at the first frame (connect ok but no data):

    // 1. Press the Processing stop button,
    // 2. Unplug the usb wire from the Teensy, and plug it back in. Note processing display window closes.
    // 3. If the serial port connected to Teensy is not visible in Arduino Tools >> Port, 
    //    click "Verify" on Arduino to compile the code, then press the little button on the Teensy.
    //    Note that the Arduino menu >> Tools >> Port only updates when you re-select it after displaying other menus.
    // 4. Open the Arduino IDE and then the Arduino serial monitor, and then close it.
    // 5. Unplug the usb wire again, and plug it back in. Note the serial monitro window closes
    // 6. Try to run this sketch again, the frames should now be updating and plotting incoming data. 

    // The above steps were the only way I was able to work around this bug when it happened.
    // The bug is mentioned in the link above with the following description:

    // "Windows & Linux: When using the Serial Monitor with the USB Keyboard/Mouse option, 
    // sometimes a "teensy gateway communication error" can occur. 
    // Close and reopen the serial monitor to regain communication." 

    // On Linux, using Teensy serialPortName = "/dev/ttyACM0"
    // On Linux, using Arduino Uno serialPortName = "/dev/ttyUSB0"
    // On Windows, serialPortName = "COM3", "COM4", or "COM5" usually

    SerialThread serialReader = new SerialThread(this);

    // Start a seperate thread which runs in the background and reads incoming serial port data
    // serialReader.start arguments are: (String serialPortName, int serialBaudRate)
    // serialBaudRate 12500000 for Teensy 3.X
    //  serialBaudRate 1000000 for Arduino running at 16 Mhz
    serialReader.start("/dev/ttyACM0", 12500000);
  }

  // Video Camera Mode =============================================================
  if (signalSource == 5) {
    noLoop();
    frameRate(60); // cheap cameras are 30, but we aim a little higher so internals are not 'lazy'
  }
  // ===============================================================================
}

void captureEvent(Capture video) {
  video.read();
  redraw();
}

void draw() { 
  drawHasFinished = false;
  //chartRedraws++;

  //if (chartRedraws >= 60) {
  //  chartRedraws = 0;
  // save a sensor data frame to a text file every 60 sensor frames
  //String[] stringArray = new String[SENSOR_PIXELS];
  //for(outerPtrX=0; outerPtrX < SENSOR_PIXELS; outerPtrX++) { 
  //   stringArray[outerPtrX] = str(output[outerPtrX]);
  //}
  //   saveStrings("Pixel_Values.txt", stringArray);
  //}

  // Plot the Data using the DataPlot object
  DP1.drawPlot();
  drawHasFinished = true;
}

void keyPressed() {
  DP1.dpkeyPressed();
}

void mouseDragged() {
  DP1.dpmouseDragged();
}

void mouseWheel(MouseEvent event) {
  DP1.dpmouseWheel(-event.getCount()); // note the minus sign (-) inverts the mouse wheel output direction
}