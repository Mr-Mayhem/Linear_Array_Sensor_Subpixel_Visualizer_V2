class dataPlot {
  // Written by Douglas Mayhew 12/1/2016
  // Plots data and provides mouse sliding and zooming ability
  // ==============================================================================================
  // colors

  /* =================================== Peak Finder Notes ========================================
   Finds all shadows or simulated shadows as pairs of peaks in the 1st difference data,
   and reports the shadow position with subpixel accuracy using quadratic interpolation.
   
   We skip the first KERNEL_LENGTH of convolution output data, which is garbage from the 
   convolution kernel not being fully immersed in the input data.
   
   Index pointer housekeeping math can drive you crazy, heh... Let me try to explain the pieces:
   
   Our goal here is to correctly draw and report the peak locations relative to the original data 
   in the time axis of the plot, the x axis.
   
   1. Speaking relative to the outer loop pointer, outerPtrX, the center-peak 
   pixel value, diff1Y, was set one outer loop increment ago, so the reported peak x index 
   location would normally be (outerPtrX - 1), to shift all reported peak positions 
   negative by 1.
   
   We also consider that we want to report the 1st sensor pixel as pixel 1, not zero, so the 
   reported peak x index location would be (outerPtrX + 1) to shift all reported peak positions 
   positive by 1.
   
   To resolve this tug-of-war situation, we simply let the -1 and the +1 opposites cancel by 
   ignoring both, so at this stage of thinking, the peak location is reported as outerPtrX with 
   no changes.
   
   2. Remember the first difference points in this case are the subtracted difference between a data 
   point value and it's lower index neighbor, so we subtract 0.5 from outerPtrX to shift the
   reported peak locations down by -0.5 to reflect this. This correction appears as 
   outerPtrX - 0.5 below.
   
   3. We compensate for the positive index shift that occurs during convolution by shifting left 
   by half a kernel-length in the opposite (negative) direction. This correction appears as 
   outerPtrX - HALF_KERNEL_LENGTH below.
   
   4. The combination of all the above considerations is: 
   peakLoc = (outerPtrX - 0.5) - HALF_KERNEL_LENGTH
   
   This correctly draws and reports the peak locations relative to the original data in the 
   time axis of the plot. There are probably better ways to further reduce this problem, but
   this worked for me. I intend to update the technique if and when I learn new peak finding tricks.
   */

  final color COLOR_ORIGINAL_DATA = color(255);
  final color COLOR_KERNEL_DATA = color(255, 255, 0);
  final color COLOR_FIRST_DIFFERENCE = color(0, 255, 0);
  final color COLOR_OUTPUT_DATA = color(255, 0, 255);
  final color COLOR_NEGATIVE_TICK = color(0, 0, 255);
  final color COLOR_CENTER_TICK = color(255, 255, 255);
  final color COLOR_POSITIVE_TICK = color(255, 0, 0);
  final color COLOR_NEGATIVE_PEAK_CLUSTER = color(0, 0, 255);
  final color COLOR_POSITIVE_PEAK_CLUSTER = color(255, 0, 0);

  // Y Positions (height-wise positions) of display bars which show various signal data along top of screen
  final int COLORBAR_1_Y = 0;
  final int COLORBAR_2_Y = 10;
  final int COLORBAR_3_Y = 20;
  final int COLORBARS_TOTAL_HEIGHT = 30;

  // other constants
  final float sensorPixelSpacingX = 0.0635;           // 63.5 microns
  final float sensorPixelsPerMM = 15.74803149606299;  // number of pixels per mm in sensor TSL1402R and TSL1410R
  final float sensorWidthAllPixels = 16.256;          // millimeters

  // ==============================================================================================
  int dpXpos;                // dataPlot class init variables
  int dpYpos;
  int dpWidth;
  int dpHeight;
  int dpDataLen;
  int dpTextSize;
  int dptextSizePlus2;       // used to space text height-wise

  int adcBitDepth;
  int highestADCValue; 

  float input;               // The input data value of a given sensor pixel
  // using a float here lets us use interpolation to simulate shadow movement in simulation modes.

  float cOutPrev;            // the previous convolution output y value
  float cOut;                // the current convolution output y value

  float kernelMultiplier;    // multiplies the plotted kernel values for greater visibility because the values are small
  int kernelDrawYOffset;     // height above bottom of screen to draw the kernel data points

  int wDataStartPos;         // the index of the first data point
  int wDataStopPos;          // the index of the last data point

  int outerPtrX;             // outer loop pointer
  int outerCount;            // outer loop counter
  int innerPtrX;             // inner loop pointer, used only during convolution

  float pan_x;               // local copies of variables from PanZoom object
  float scale_x;
  float pan_y;
  float scale_y;
  float dpKernelSigma;       // current kernel sigma as determined by kernel Pan Zoom object
  float dpPrevKernelSigma;   // previous kernel sigma as determined by kernel Pan Zoom object
  float drawPtrX;            // phase correction drawing pointers
  float drawPtrXLessK;       // x axis value for drawing the points in phase after convolution
  float drawPtrXLessKlessD1; // same as above but - half a pixel for derivative correction

  boolean modulateX;         // toggles the modulation on and off
  int modulationIndex;       // index to index through the modulation waveform
  float modulationX;         // added to x axis of data to simulate left right movement
  float lerpX;               // new y value in between adjacent x's, used to shift data in input array to simulate shadow motion
  int offsetY;               // temp variable to offset the y position (the height) of text
  int widthMinus1;           // width - 1, reduces math work
  float frameRateLowPass;    // used to smooth the frame rate text display jitter

  // =============================================================================================
  // Subpixel Variables

  float negPeakX;            // x index position of greatest negative y difference peak found in 1st difference data
  float posPeakX;            // x index position of greatest positive y difference peak found in 1st difference data

  float pixelWidthX;         // integer difference between the two peaks without subpixel precision

  float negPeakLeftY;        // y value of left neighbor (x - 1) of greatest 1st difference negative peak
  float negPeakCenterY;      // y value of 1st difference (x) of greatest negative peak
  float negPeakRightY;       // y value of right neighbor (x + 1) of greatest 1st difference negative peak

  float posPeakLeftY;        // y value of left neighbor (x - 1) of greatest 1st difference positive peak
  float posPeakCenterY;      // y value of 1st difference (x) of greatest positive peak
  float posPeakRightY;       // y value of right neighbor (x + 1) of greatest 1st difference positive peak

  float negPeakSubPixelX;    // quadratic interpolated negative peak subpixel x position
  float posPeakSubPixelX;    // quadratic interpolated positive peak subpixel x position

  float preciseWidthX;       // precise shadow width (in pixels with subpixels after decimal point)
  float preciseWidthLowPassX;// precise shadow width filtered with simple running average filter
  float preciseWidthXmm;     // precise shadow width converted to mm

  float preciseCenterPositionX;   // center position output in pixels
  float preciseCenterPositionXlp; // precise center position filtered with simple running average filter
  float preciseCenterPositionXmm; // precise center position converted to mm

  float shiftSumX;           // temporary variable for summing x shift values
  float CalCoefWidthX;       // corrects mm width by multiplying by this value

  // diff0Y holds the difference between the current convolution output value and the previous one, 
  // in the form y[x] - y[x-1]
  // If a peak is found in the difference data, this becomes the right side value of the 3 peak values 
  // which bracket the peak and feed the quadratic interpolation function, which in turn finds the 
  // subpixel location.
  float diff0Y; 

  // diff1Y holds the difference between the previous convolution output value and the one prior, 
  // in the form y[x-1] - y[x-2]
  // If a peak is found in the difference data, this becomes the center value of the 3 peak values 
  // which bracket the peak and feed the quadratic interpolation function, which in turn finds the 
  // subpixel location.
  float diff1Y;

  // diff2Y holds the difference between the second prior convolution output value and the one prior, 
  // in the form y[x-2] - y[x-3]
  // If a peak is found in the difference data, this becomes the left side value of the 3 peak values 
  // which bracket the peak and feed the quadratic interpolation function, which in turn finds the 
  // subpixel location.
  float diff2Y;

  float ScreenNegX;          // holds screen X coordinate for the negative peak subpixel position
  float ScreenCenX;          // holds screen X coordinate for the center subpixel position
  float ScreenPosX;          // holds screen X coordinate for the positive peak subpixel position

  float ScreenMarkX;         // holds screen X coordinate for the center 3 pixel marker subpixel position

  int markSize;              // diameter of drawn subpixel marker circles
  int subpixelMarkerLenY;    // length of vertical lines which indicate subpixel peaks and shadow center location
  int movingAvgKernalLenX;   // Length of moving average filter used to smooth subpixel output

  int diffThresholdY;        // threshold below which peaks are ignored in the 1st difference peak finder

  int detCounter;            // counts the number of subpixel runs in one frame of data

  boolean negPeakFound;      // set true after a negative peak is found with a positive peak, for a complete pair.
  // =============================================================================================
  PImage cameraImage;

  // =============================================================================================
  //Arrays

  // array for output signal
  float[] output = new float[0]; 

  // array for output signal
  float[] detections = new float[0]; 

  // =============================================================================================
  PanZoomX PanZoomPlot;       // pan/zoom object to control pan & zoom of main data plot
  //MovingAverageFilter F1;     // filters the subpixel output data
  Waterfall WaterFall1; 

  dataPlot (PApplet p, int plotXpos, int plotYpos, int plotWidth, int plotHeight, int plotDataLen, int bitDepth, int TextSize) {
    widthMinus1 = width - 1; // saves on math work

    dpXpos = plotXpos;
    dpYpos = plotYpos;
    dpWidth = plotWidth;
    dpHeight = plotHeight;
    dpDataLen = plotDataLen;
    dpTextSize = TextSize;
    dptextSizePlus2 = dpTextSize + 2;

    // the number of bits data values consist of
    adcBitDepth = bitDepth;

    highestADCValue = int(pow(2.0, float(adcBitDepth))-1); 
    println("highestADCValue = " + highestADCValue);

    // Create PanZoom object to pan & zoom the main data plot
    PanZoomPlot = new PanZoomX(p, plotDataLen);

    pan_x = PanZoomPlot.getPanX();  // initial pan and zoom values
    scale_x = PanZoomPlot.getScaleX();
    pan_y = PanZoomPlot.getPanY();
    scale_y = PanZoomPlot.getScaleY();

    outerPtrX = 0;         // outer loop pointer
    outerCount = 0;        // outer loop counter
    innerPtrX = 0;         // inner loop pointer, used only during convolution

    // multiplies the plotted y values of the kernel, for greater height 
    // visibility since the values in typical kernels are so small
    kernelMultiplier = 100.0;

    // height above bottom of screen to draw the kernel data points                                      
    kernelDrawYOffset = 75; 

    // diameter of drawn subpixel marker circles
    markSize = 3;

    // sets height deviation of vertical lines from center height, 
    // indicates subpixel peaks and shadow center location
    subpixelMarkerLenY = int(height * 0.02);

    // threshold below which peaks are ignored in the 1st difference peak finder
    diffThresholdY = 128;

    // corrects mm width by multiplying by this value
    CalCoefWidthX = 0.981;

    // array for convolution, get resized after KERNEL_LENGTH is known,
    // must always match kernel length

    output = new float[KERNEL_LENGTH];

    detections = new float[dpDataLen];  // holds subpixel center positions

    WaterFall1 = new Waterfall(p);
    WaterFall1.init(width, round(height/2.5));

    // init the camera image if using that mode
    if (signalSource == 5) {
      cameraImage = createImage(640, 60, RGB);
    }

    //movingAvgKernalLenX = 7;
    //// reduces jitter on subpixel preciseCenterPositionX, higher = more smoothing, default = 7;
    //F1 = new MovingAverageFilter(movingAvgKernalLenX);
  }

  boolean overKernel() {
    if (mouseX > 0 && mouseX < dpWidth && 
      mouseY > height - 120) {
      return true;
    } else {
      return false;
    }
  }

  boolean overPlot() {
    if (mouseX > 0 && mouseX < dpWidth && 
      mouseY > 0 && mouseY < height - 120) {
      return true;
    } else {
      return false;
    }
  }

  void dpkeyPressed() { // we simply pass through the mouse events to the pan zoom object
    PanZoomPlot.pzkeyPressed();
  }

  void dpmouseDragged() {
    if (overPlot()) {
      PanZoomPlot.pzmouseDragged();
    }
  }

  void dpmouseWheel(int step) {
    if (overKernel()) {
      // we may change the kernel size and output array size, so to prevent array index errors, 
      // set the outer loop pointer to the last value it would normally reach during normal operation
      outerPtrX = wDataStopPos-1;  
      SG1.sgmouseWheel(step);  // this passes to the kernel generator which makes the new kernel array on the fly
      output = new float[KERNEL_LENGTH];  // this sizes the output array to match the new kernel array length
    } else if (overPlot()) {
      PanZoomPlot.pzmouseWheel(step);
    }
  }

  void drawPlot() { // outer draw loop in this class
    background(0);

    outerCount = 0;          // outer loop counter, increments once each outer loop
    detCounter = 0;          // counts the number of subpixel shadows

    // update the local pan and scale variables from the PanZoom object which maintains them
    pan_x = PanZoomPlot.getPanX();
    scale_x = PanZoomPlot.getScaleX();
    pan_y = PanZoomPlot.getPanY();
    scale_y = PanZoomPlot.getScaleY();

    drawGrid2(pan_x, ((wDataStopPos) * scale_x) + pan_x, 0, height-1 + pan_y, scale_x, scale_y);

    // The minimum number of input data samples is two times the kernel length + 1,  which results in 
    // the minumum of only one sample processed. (we ignore the fist and last data by one kernel's width)
    if (modulateX) {
      modulationIndex++; // increment the index to the sine wave array
      if (modulationIndex > sineArray.length-1) {
        modulationIndex = 0;
      }
      modulationX = sineArray[modulationIndex]; // value used to interpolate the data 
      // to simulate shadow movement
    }

    wDataStartPos = 0;
    wDataStopPos = dpDataLen;

    //wDataStartPos = constrain(wDataStartPos, 0, dpDataLen);
    //wDataStopPos = constrain(wDataStopPos, 0, dpDataLen);

    if (signalSource == 3) {

      // Plot using Serial data, remember to plug in Teensy 3.6 via usb programming cable 
      // and that sister sketch is running

      processSerialSensorData();
    } else if (signalSource == 5) { 

      // Plot using Video data, make sure at least one camera is connected and enabled, 
      // resolution default set to 640 x 480
      video.loadPixels();
      cameraImage.loadPixels();
      wDataStartPos = 0;
      wDataStopPos = video.width;

      // Copy rowToCopy pixels from the video and write them to videoData array, 
      // which holds one row of video pixel values
      int rowToCopy = video.height/2;
      int firstPixel = (rowToCopy * video.width);

      //println(firstPixel);
      for (int x = 0; x < SENSOR_PIXELS; x++) {  // copy one row of video data to the video array
        int index = (firstPixel + x);
        videoArray[x] = video.pixels[index];     // copy pixel
        // color pixel green so we can see where the row is in the video display
        video.pixels[index] = color(0, 255, 0);
      }

      video.updatePixels();
      int srcYOffset = rowToCopy -30;
      int dest_w = cameraImage.width;
      int dest_h = cameraImage.height;
      // we don't want to display the entire camera image, 
      // just the area vertically near the row we are using
      // copy the center 120 pixels from the video to the cameraImage

      for (int y = 0; y < dest_h; y++) {                  // rows top down
        for (int x = 0; x < dest_w; x++) {                // columns left to right
          int setPixelIndex = (y * dest_w) + x;           // pixel source index  
          int getPixelIndex = ((y + srcYOffset) * dest_w)+x;   // pixel dest index
          cameraImage.pixels[setPixelIndex] = video.pixels[getPixelIndex];
        }
      }

      cameraImage.updatePixels();
      float x = (cameraImage.width * scale_x);
      image(cameraImage, pan_x, 30, x, cameraImage.height);

      processVideoData();
    } else {
      // Plot using Simulated Data
      processSignalGeneratorData();
    } 

    WaterFall1.refresh(); // update and draw the new waterfall image

    refreshShadowInfoText();
    //offsetY = height - 140;
    //fill(255);
    //textAlign(CENTER);
    //text("Use mouse to drag, mouse wheel to zoom", HALF_SCREEN_WIDTH, offsetY);

    //offsetY += dptextSizePlus2;
    //text("pan_x: " + String.format("%.3f", pan_x) + "  scale_x: " 
    //  + String.format("%.3f", scale_x), HALF_SCREEN_WIDTH, offsetY);
    //textAlign(LEFT);

    // Counts 1 to 60 and repeats, to provide a sense of the frame rate
    // text(chartRedraws, 10, 50);

    // draw actual frameRate
    frameRateLowPass = (frameRateLowPass * 0.99) + (frameRate * 0.01); // apply a simple low pass filter
    text("Frame Rate: " + String.format("%.1f", frameRateLowPass), 10, 40);

    // draw Legend and Kernel
    drawLegend();
    drawKernel(0, SG1.kSigma);

    //// list the subpixel detections scross the screen for debugging
    //for (outerPtrX = 0; outerPtrX < dpDataLen; outerPtrX++) {
    //  if (detections[outerPtrX] > 0) {
    //    text("[" + outerPtrX + "]:" + String.format("%.3f", detections[outerPtrX]), outerPtrX*70, height-(height/3));

    //    // erase the previous detection value so it does not linger in the array between frames
    //    // we do this here rather than in the outer loop for efficiency; no need to set all elements to zero,
    //    // because there are many less detections than array elements.
    //    detections[outerPtrX] = 0;
    //  } else {
    //    break; // there should not be any valid entries after the first zero entry so quit here for efficiency
    //  }
    //}
  }

  void drawKernel(float pan_x, double sigma) {

    // plot kernel data point
    stroke(COLOR_KERNEL_DATA);

    for (outerPtrX = 0; outerPtrX < KERNEL_LENGTH; outerPtrX++) { 
      // shift outerPtrX left by half the kernel size to correct for convolution shift 
      // (dead-on correct for odd-size kernels)
      drawPtrXLessK = ((outerPtrX - HALF_KERNEL_LENGTH) * scale_x) + pan_x; 

      // draw new kernel point (y scaled up by kernelMultiplier for better visibility)
      point(drawPtrXLessK+HALF_SCREEN_WIDTH, 
        height-kernelDrawYOffset - (kernel[outerPtrX] * kernelMultiplier));
    }

    fill(255);
    offsetY = height-40;
    textAlign(CENTER);
    text("Use mouse wheel here to adjust kernel", HALF_SCREEN_WIDTH, offsetY);

    offsetY -= dptextSizePlus2;
    text("Kernel Sigma: " + String.format("%.2f", sigma), HALF_SCREEN_WIDTH, offsetY);

    offsetY -= dptextSizePlus2;
    text("Kernel Length: " + KERNEL_LENGTH, HALF_SCREEN_WIDTH, offsetY);
    textAlign(LEFT);
  }

  void processSerialSensorData() { // synchronized with Serial_Thread which reads serial port in a seperate thread
    // draw text showing the number of Buffered Bytes in the serial buffer, shows how well we are keeping up

    text("Serial Bytes Available: " + bytesAvailable, 10, 60);
    text("Serial Bits Per Sec: " + nfc(bitRate, 0), 10, 70);
   
    text("Serial Frames Per Sec: " + nfc(dataFrameRate, 0), 10, 90);
    text("Shadow Info Frames Per Sec: " + nfc(shadowFrameRate, 0), 10, 100);
    
    text("Teensy Shadow Center: " + nf(subpixelCenter), 10, 120);
    text("Teensy Shadow Width:   " + nf(subpixelWidth), 10, 130);

    // increment the outer loop pointer
    for (outerPtrX = wDataStartPos; outerPtrX < wDataStopPos; outerPtrX++) {
      outerCount++; // lets us index (x axis) on the screen offset from outerPtrX

      // Below we prepare 3 shifted X axis indexes for plotting on the screen

      // the outer counter, scaled and panned for the screen
      drawPtrX = (outerCount * scale_x) + pan_x;

      // shift left by half the kernel size to correct for convolution shift 
      // (dead-on correct for odd-size kernels)
      drawPtrXLessK = drawPtrX - (HALF_KERNEL_LENGTH * scale_x); 

      // same as above, but shift left additional 0.5 to properly place the 1st difference 
      // point in-between it's parents
      drawPtrXLessKlessD1 = drawPtrXLessK - (scale_x * 0.5);

      // parse two pixel data values from the serial port data byte array:
      // Read a pair of bytes from the byte array, convert them into an integer, 
      // shift right 2 places(divide by 4), and copy the value to a simple global variable

      // original data input value
      //input = (byteArray[outerPtrX<<1]<< 8 | (byteArray[(outerPtrX<<1) + 1] & 0xFF))>>2;

      input = (byteArrayDecoded[outerPtrX<<1]<< 8 | (byteArrayDecoded[(outerPtrX<<1) + 1] & 0xFF));
      //input = int(map(input, 0, highestADCValue, 0, height * 8));
      //if (!modulateX){
      //  input = (byteArray[outerPtrX<<1]<< 8 | (byteArray[(outerPtrX<<1) + 1] & 0xFF))>>2;
      //}else{
      //  input = lerp((byteArray[outerPtrX<<1]<< 8 | (byteArray[(outerPtrX<<1) + 1] & 0xFF))>>2, 
      //  (byteArray[(outerPtrX+1)<<1]<< 8 | (byteArray[((outerPtrX+1)<<1) + 1] & 0xFF))>>2, modulationX);
      //}

      // plot original data value
      stroke(COLOR_ORIGINAL_DATA);

      point(drawPtrX, HALF_SCREEN_HEIGHT - (input * scale_y));

      // number the pixels for testing to see if they are all there
      //fill(255);
      //text(outerCount, drawPtrX, (HALF_SCREEN_HEIGHT - (input * scale_y)) - 10 );

      // draw one rectangular piece of greyscale bar representing original data
      drawColorBarSegment(drawPtrX, COLORBAR_1_Y, input);

      convolutionInnerLoop(); // Convolution Inner Loop

      // Skip one kernel length of convolution output values, which are garbage.
      if (outerCount > KERNEL_LENGTH_MINUS1) {  
        // plot the output data value
        stroke(COLOR_OUTPUT_DATA);
        point(drawPtrXLessK, HALF_SCREEN_HEIGHT - (cOut * scale_y));
        //println("output[" + outerPtrX + "]" +output[outerPtrX]);

        // draw one rectangular piece of greyscale bar representing convolution output data
        drawColorBarSegment(drawPtrXLessK, COLORBAR_2_Y, cOut);

        find1stDiffPeaks();
      }
    }
  }

  void processVideoData() {

    // increment the outer loop pointer
    for (outerPtrX = wDataStartPos; outerPtrX < wDataStopPos; outerPtrX++) {
      outerCount++; // lets us index (x axis) on the screen offset from outerPtrX

      // Below we prepare 3 shifted X axis indexes for plotting on the screen

      // the outer counter, scaled and panned for the screen
      drawPtrX = (outerCount * scale_x) + pan_x;

      // shift left by half the kernel size to correct for convolution shift 
      //(dead-on correct for odd-size kernels)
      drawPtrXLessK = drawPtrX - (HALF_KERNEL_LENGTH * scale_x); 

      // same as above, but shift left additional 0.5 to properly place the difference 
      //point in-between it's parents
      drawPtrXLessKlessD1 = drawPtrXLessK - (scale_x * 0.5);

      // copy one data value from the video array, which contains one row of color video 
      // taken from the middle-height of the video, as integers convert color pixel to 
      // greyscale, and multiply by 8 to bring the levels up from 255 max to a few 
      // thousand max. 
      // original data input value

      // 3 camera values of 255 max
      input = Pixelbrightness(videoArray[outerPtrX]) * 16; 

      // plot original data value
      stroke(COLOR_ORIGINAL_DATA);

      point(drawPtrX, HALF_SCREEN_HEIGHT - (input * scale_y));

      // draw one rectangular piece of greyscale bar representing original data
      drawColorBarSegment(drawPtrX, COLORBAR_1_Y, input);

      convolutionInnerLoop(); // Convolution Inner Loop

      // Skip one kernel length of convolution output values, which are garbage.
      if (outerCount > KERNEL_LENGTH_MINUS1) {  
        // plot the output data value
        stroke(COLOR_OUTPUT_DATA);
        point(drawPtrXLessK, HALF_SCREEN_HEIGHT - (cOut * scale_y));
        //println("output[" + outerPtrX + "]" +output[outerPtrX]);

        // draw one rectangular piece of greyscale bar representing convolution output data
        drawColorBarSegment(drawPtrXLessK, COLORBAR_2_Y, cOut);

        find1stDiffPeaks();
      }
    }
  }

  void processSignalGeneratorData() {

    // increment the outer loop pointer
    for (outerPtrX = wDataStartPos; outerPtrX < wDataStopPos-1; outerPtrX++) { 
      outerCount++; // lets us index (x axis) on the screen offset from outerPtrX

      // Below we prepare 3 shifted X axis indexes for plotting on the screen

      // the outer counter, scaled and panned for the screen
      drawPtrX = (outerCount * scale_x) + pan_x;

      // shift left by half the kernel size to correct for convolution shift (dead-on correct for odd-size kernels)
      drawPtrXLessK = drawPtrX - (HALF_KERNEL_LENGTH * scale_x); 

      // same as above, but shift left additional 0.5 to properly place the difference point in-between it's parents
      drawPtrXLessKlessD1 = drawPtrXLessK - (scale_x * 0.5);

      // copy one data value from the signal generator output array:
      if (!modulateX) {
        input = sigGenOutput[outerPtrX];
      } else {
        input = LinearInterpolate(sigGenOutput[outerPtrX], sigGenOutput[outerPtrX+1], modulationX);
      }

      // plot original data value
      strokeWeight(1);
      stroke(COLOR_ORIGINAL_DATA);

      point(drawPtrX, HALF_SCREEN_HEIGHT - (input * scale_y));

      // draw one rectangular piece of greyscale bar representing original data
      drawColorBarSegment(drawPtrX, COLORBAR_1_Y, input);

      convolutionInnerLoop(); // Convolution Inner Loop

      if (outerCount > KERNEL_LENGTH_MINUS1) {  // Skip one kernel length of convolution output values, 
        // which are garbage due to the kernel not being fully immersed in the input signal.
        // plot the output data value
        stroke(COLOR_OUTPUT_DATA);
        point(drawPtrXLessK, HALF_SCREEN_HEIGHT - (cOut * scale_y));
        //println("output[" + outerPtrX + "]" +output[outerPtrX]);

        // draw one rectangular piece of greyscale bar representing convolution output data
        drawColorBarSegment(drawPtrXLessK, COLORBAR_2_Y, cOut);

        find1stDiffPeaks();
      }
    }
  }

  void convolutionInnerLoop() {
    // ================================= Convolution Inner Loop  =============================================
    // I 'invented' this convolution algorithm during experimentation in December 2016. Inner loops have probably been 
    // done this way many times before, I don't know for sure, but I haven't seen it yet in books or papers on the subject, 
    // but then again, I just recently started to play with dsp and haven't done an exhaustive search for it elsewhere. 
    // Regardless, I am proud of independently creating this little inner 1-dimentional convolution algorithm; I did not 
    // copy it from a book or the internet, it emerged from a series of what-if experiments I did.

    // This convolution machine creates one output value for each input data value (each increment of the outer loop).
    // It is unique in that it uses an output array of the same size as the kernel, rather than a larger size. 
    // One advantage is that all output[] values get overwritten for each outer loop count, without the need to 
    // zero them in a seperate step. The kernel length can be easily changed before processing a frame of data.
    // The output array size should always equal the kernel array size. Final output comes from output[0].

    cOutPrev = cOut; // y[output-1] (the previous convolution output value)

    for (innerPtrX = 0; innerPtrX < KERNEL_LENGTH_MINUS1; innerPtrX++) {     // increment the inner loop pointer
      output[innerPtrX] = output[innerPtrX+1] + (input * kernel[innerPtrX]); // convolution: multiply and accumulate
    }

    output[KERNEL_LENGTH_MINUS1] = input * kernel[KERNEL_LENGTH_MINUS1];     // convolution: multiply only, no accumulate     

    cOut = output[0]; // y[output] (the current convolution output value)

    // To make this convolution inner loop easier to understand, I unwrap the loop below.
    // If you run it, remember to comment out the original loop-based convolution code above
    // or you will convolve the input data twice, a waste of CPU cycles.
    // The unwrapped loop code below runs a little faster, but if you change kernel length,
    // match it by adding or removing code lines below.

    //cOutPrev = cOut; // y[output-1] (the previous convolution output value)

    //output[0] = output[1] + (input * kernel[0]); // 1st kernel point, convolution: multiply and accumulate
    //output[1] = output[2] + (input * kernel[1]); // 2nd kernel point, convolution: multiply and accumulate
    //output[2] = output[3] + (input * kernel[2]); // 3rd kernel point, convolution: multiply and accumulate
    //output[3] = output[4] + (input * kernel[3]); // 4th kernel point, convolution: multiply and accumulate
    //output[4] = output[5] + (input * kernel[4]); // 5th kernel point, convolution: multiply and accumulate
    //output[5] = output[6] + (input * kernel[5]); // 6th kernel point, convolution: multiply and accumulate
    //output[6] = output[7] + (input * kernel[6]); // 7th kernel point, convolution: multiply and accumulate
    //output[7] = output[8] + (input * kernel[7]); // 8th kernel point, convolution: multiply and accumulate

    //output[8] = input * kernel[8]; // 9th kernel point, convolution: multiply only, no accumulate

    //cOut = output[0]; // y[output] (the current convolution output value)

    // ==================================== End Convolution ==================================================
  }

  void find1stDiffPeaks() {
    // =================== Find the 1st difference and store the last two values  ==========================
    // finds the differences and maintains a history of the previous 2 difference values as well,
    // so we can collect all 3 points bracketing a pos or neg peak, needed to feed the subpixel code.

    diff2Y=diff1Y;      // (left y value)
    diff1Y=diff0Y;      // (center y value)  
    // find 1st difference of the convolved data, the difference between adjacent points in the smoothed data.
    diff0Y = cOut - cOutPrev; // (right y value) // << The first difference is the difference between the current 
    // convolution output value and the previous one, in the form y[x] - y[x-1]
    // In dsp, this difference is preferably called the "first difference", but some texts call it the 
    // "first derivative", and some texts refer to each difference value produced above as a "partial derivative".

    // ====================================== End 1st difference ============================================

    // plot the first difference data value
    stroke(COLOR_FIRST_DIFFERENCE);
    point((drawPtrXLessKlessD1), HALF_SCREEN_HEIGHT - (diff0Y * scale_y));

    // draw one rectangular piece of greyscale bar representing 1st difference
    drawColorBarSegmentAbs((drawPtrXLessKlessD1), COLORBAR_3_Y, diff0Y);

    // ======================================== Peak Finder =================================================

    if (abs (diff1Y) > diffThresholdY) { // if the absolute value of the peak is above the threshold value
      if (diff1Y < diff0Y && diff1Y < diff2Y) { // if diff1Y is a negative peak relative to the neighboring values
        negPeakX = (outerPtrX - 0.5) - HALF_KERNEL_LENGTH; // x-1 and x-0.5 for difference being in-between original data
        negPeakRightY = diff0Y;   // y value @ x index -1 (right)
        negPeakCenterY = diff1Y;  // y value @ x index -2 (center) (negative 1st difference peak location)
        negPeakLeftY = diff2Y;    // y value @ x index -3 (left)
        negPeakFound = true;
      } else if (diff1Y > diff0Y && diff1Y > diff2Y) { // if diff1Y is a positive peak relative to the neighboring values
        posPeakX = (outerPtrX - 0.5) - HALF_KERNEL_LENGTH;
        posPeakRightY = diff0Y;   // y value @ x index -1 (right)
        posPeakCenterY = diff1Y;  // y value @ x index -2 (center) (positive 1st difference peak location)
        posPeakLeftY = diff2Y;    // y value @ x index -3 (left)
        if (negPeakFound) { // insures that pairs of peaks(one negative, one positive) are fed to subpixelCalc
          subpixelCalc(); // calculate, display, and store the subpixel estimate associated with this peak pair
          negPeakFound = false; // reset for next time around

          negPeakX = 0;
          negPeakRightY = 0;
          negPeakCenterY = 0;
          negPeakLeftY = 0;

          posPeakX = 0;
          posPeakRightY = 0;
          posPeakCenterY = 0;
          posPeakLeftY = 0;
        }
      }
    }
  }

  void subpixelCalc() {

    // we should have already ran a gaussian smoothing routine over the data, and 
    // found the x location and y values for the positive and negative peaks of the first differences,
    // and the neighboring first differences immediately to the left and right of these on the x axis.
    // Therefore, all we have remaining to do, is the quadratic interpolation routines and the actual 
    // drawing of their positions, after a quality check of the peak heights and width between them.

    // the subpixel location of a shadow edge is found as the peak of a parabola fitted to 
    // the top 3 points of a smoothed original data's first difference peak.

    // the first difference is simply the difference between adjacent data 
    // points of the original data, ie, 1st difference = x[i] - x[i-1], for each i.

    // Each difference value is proportional to the steepness and direction of the slope in the 
    // original data between the two original points in question. (smoothed original)
    // Also in this case we smooth the original data first to make the peaks we are searching for
    // more symmectrical and rounded, and thus closer to the shape of a parabola, which we fit to 
    // the peaks next. The more the highest (or lowest for negative peaks) 3 points of the peaks 
    // resemble a parabola, the more accurate the subpixel result.

    pixelWidthX=posPeakX-negPeakX;

    // check for width in acceptable range, what is acceptable is up to you, within reason.
    // was originally 'pixelWidthX < 103' for filiment width sketch, (15.7pixels per mm, 65535/635=103)
    if (pixelWidthX > 8 && pixelWidthX < 256) { // if pixel-based width is within this range

      // sub-pixel edge detection using interpolation
      // from Accelerated Image Processing blog, posting: Sub-Pixel Maximum
      // https://visionexperts.blogspot.com/2009/03/sub-pixel-maximum.html

      // for the subpixel value of the greatest negative peak found above, 
      // corresponds with the left edge of a shadow cast upon the sensor
      negPeakSubPixelX = 0.5 * (negPeakLeftY - negPeakRightY) / (negPeakLeftY - (2 * negPeakCenterY) + negPeakRightY);

      // for the subpixel value of the greatest positive peak found above, 
      // corresponds with the right edge of a shadow cast upon the sensor
      posPeakSubPixelX = 0.5 * (posPeakLeftY - posPeakRightY) / (posPeakLeftY - (2 * posPeakCenterY) + posPeakRightY);

      // original function translated from flipper's filament width sensor; 
      // it does the same math calculation as above but using more division
      // better for a fixed-point calculation where there is no 0.5 possible
      // negPeakSubPixelX=((a1-c1) / (a1+c1-(b1*2)))/2;
      // posPeakSubPixelX=((a2-c2) / (a2+c2-(b2*2)))/2;

      preciseWidthX = pixelWidthX + (posPeakSubPixelX - negPeakSubPixelX);

      //preciseWidthLowPassX = (preciseWidthLowPassX * 0.9) + (preciseWidthX * 0.1); // apply a simple low pass filter
      preciseWidthXmm = preciseWidthX * sensorPixelSpacingX * CalCoefWidthX;

      // solve for the center position by adding the left and right pixel and subpixel locations up, and then dividing the sum by 2
      preciseCenterPositionX = (((negPeakX + negPeakSubPixelX) + (posPeakX + posPeakSubPixelX)) / 2);

      // copy the subpixel center value to the detections array, useful downstream for multi-shadow calculations & averaging
      detections[detCounter] = preciseCenterPositionX; 
      // increment the detection counter
      detCounter++; 

      //F1.nextValue(preciseCenterPositionX);
      //preciseCenterPositionXlp = F1.getAverage();

      // keep the center from lagging too far; keep it between the left and right peaks during rapid shadow movement
      //preciseCenterPositionXlp = constrainFloat(preciseCenterPositionXlp, negPeakX, posPeakX);

      //preciseCenterPositionXlp = (preciseCenterPositionXlp * 0.9) + (preciseCenterPositionX * 0.1);         // apply a simple low pass filter

      preciseCenterPositionXmm = preciseCenterPositionX * sensorPixelSpacingX;

      noFill();

      // Mark negative 1st difference subpixel estimate with line
      ScreenNegX = ((negPeakX + negPeakSubPixelX - wDataStartPos) * scale_x) + pan_x;
      if (ScreenNegX > 0 && ScreenNegX < widthMinus1) { // if the x index is on the screen)
        stroke(COLOR_NEGATIVE_TICK);
        line(ScreenNegX, HALF_SCREEN_HEIGHT + subpixelMarkerLenY, ScreenNegX, HALF_SCREEN_HEIGHT - subpixelMarkerLenY);
        line(ScreenNegX, 0, ScreenNegX, COLORBARS_TOTAL_HEIGHT); 
        WaterFall1.waterfallTop[int(ScreenNegX)] = color(COLOR_NEGATIVE_TICK); // color the waterfall top pixel the same
      }

      // Mark shadow center subpixel resolution position with line
      ScreenCenX = ((preciseCenterPositionX - wDataStartPos) * scale_x) + pan_x;
      if (ScreenCenX > 0 && ScreenCenX < widthMinus1) { // if the x index is on the screen)
        stroke(COLOR_CENTER_TICK);
        line(ScreenCenX, HALF_SCREEN_HEIGHT + subpixelMarkerLenY, ScreenCenX, HALF_SCREEN_HEIGHT - subpixelMarkerLenY); 
        line(ScreenCenX, 0, ScreenCenX, COLORBARS_TOTAL_HEIGHT); 
        WaterFall1.waterfallTop[int(ScreenCenX)] = color(COLOR_CENTER_TICK); // color the waterfall top pixel the same
        fill(COLOR_CENTER_TICK);
        textAlign(CENTER); // center the center position text over the white center marker pip line
        text(detCounter, int(ScreenCenX), HALF_SCREEN_HEIGHT - subpixelMarkerLenY-10);
        text(String.format("%.3f", preciseCenterPositionX), int(ScreenCenX), HALF_SCREEN_HEIGHT - subpixelMarkerLenY-20);
        textAlign(LEFT);  // set the textAlign back to LEFT so we don't mess up text below
        noFill();
      }

      // Mark positive 1st difference subpixel estimate with line
      stroke(COLOR_POSITIVE_TICK);
      ScreenPosX = ((posPeakX + posPeakSubPixelX - wDataStartPos) * scale_x) + pan_x;
      if (ScreenPosX > 0 && ScreenPosX < widthMinus1) { // if the x index is on the screen)
        line(ScreenPosX, HALF_SCREEN_HEIGHT + subpixelMarkerLenY, ScreenPosX, HALF_SCREEN_HEIGHT - subpixelMarkerLenY);
        line(ScreenPosX, 0, ScreenPosX, COLORBARS_TOTAL_HEIGHT); 
        WaterFall1.waterfallTop[int(ScreenPosX)] = COLOR_POSITIVE_TICK; // color the waterfall top pixel the same
      }

      // Draw Ellipse on top 3 1st difference pixels

      // Mark negPeakX 3 pixel cluster with one circle each
      ScreenMarkX = ((negPeakX - wDataStartPos) * scale_x) + pan_x;
      if (ScreenNegX > 0 && ScreenNegX < widthMinus1) { // if the x index is on the screen)
        stroke(COLOR_NEGATIVE_PEAK_CLUSTER);
        ellipse(ScreenMarkX - scale_x, (HALF_SCREEN_HEIGHT - (negPeakLeftY * scale_y)), markSize, markSize);
        ellipse(ScreenMarkX, (HALF_SCREEN_HEIGHT - (negPeakCenterY * scale_y)), markSize, markSize);
        ellipse(ScreenMarkX + scale_x, (HALF_SCREEN_HEIGHT - (negPeakRightY * scale_y)), markSize, markSize);
      }

      // Mark posPeakX 3 pixel cluster with one circle each
      ScreenMarkX = ((posPeakX - wDataStartPos) * scale_x) + pan_x;
      if (ScreenMarkX > 0 && ScreenMarkX < widthMinus1) { // if the x index is on the screen)
        stroke(COLOR_POSITIVE_PEAK_CLUSTER);
        ellipse(ScreenMarkX - scale_x, (HALF_SCREEN_HEIGHT - (posPeakLeftY * scale_y)), markSize, markSize);
        ellipse(ScreenMarkX, (HALF_SCREEN_HEIGHT - (posPeakCenterY * scale_y)), markSize, markSize);
        ellipse(ScreenMarkX + scale_x, (HALF_SCREEN_HEIGHT - (posPeakRightY * scale_y)), markSize, markSize);
      }
    }
  }

  void drawColorBarSegment(float x, float y, float value) {

    // prepare color to correspond to sensor pixel reading
    int greyVal = int(map(value, 0, highestADCValue, 0, 255));

    // Plot a row of pixels near the top of the screen ,
    // and color them with the 0 to 255 greyscale sensor value
    stroke(greyVal);
    fill(greyVal);
    rect(x, y, scale_x, 9);
  }

  void drawColorBarSegmentAbs(float x, float y, float value) {
    float greyScale = map(abs(value), 0, highestADCValue/4, 0, 255);
    // prepare color to correspond to sensor pixel reading
    color greenish = color(0, greyScale, 0);  // greenish because we vary green and leave red and blue at zero
    // Plot a row of pixels near the top of the screen ,
    // and color them with the 0 to 255 greyscale sensor value

    stroke(greenish);
    fill(greenish);
    rect(x, y, scale_x, 9);
    x = constrain(x, 0, width-1);
    int xx = int(x);
    for (int i = xx; i < (xx + scale_x); i++) {
      int index = constrain(i, 0, width-1);
      WaterFall1.waterfallTop[index] = greenish; // color the waterfall top pixel the same
    }
  }

  void refreshShadowInfoText() {

    // print the text for the subpixel output values
    fill(255);
    offsetY = height-80;

    // text("SubPixel - " + nfs(negPeakSubPixelX, 0, 4), 10, offsetY);

    // offsetY -= dptextSizePlus2;
    // text("SubPixel + " + nfs(posPeakSubPixelX, 0, 4), 10, offsetY);

    // offsetY -= dptextSizePlus2;
    // text("Width: " + nf(preciseWidthX, 0, 4), 10, offsetY);

    offsetY -= dptextSizePlus2;
    text("Processing Shadow Width:    " + nf(preciseWidthXmm, 0, 4), 10, offsetY);

    // offsetY -= dptextSizePlus2;
    // text("Center: " + nf(preciseCenterPositionX, 0, 4), 10, offsetY);

    offsetY -= dptextSizePlus2;
    text("Processing Shadow Center: " + nf(preciseCenterPositionXmm, 0, 4), 10, offsetY);

    // offsetY -= dptextSizePlus2;
    // text("Number of Shadows Detected: " + detCounter, 10, offsetY);

    //offsetY -= (dptextSizePlus2) * 2;
    //text("Last Detected Shadow Info (far right)", 10, offsetY);
  }


  float constrainFloat(float value, float min, float max) {
    float retVal;
    if (value < min) {
      retVal = min;
    } else if (value > max) {
      retVal = max;
    } else {
      retVal = value;
    }
    return retVal;
  }

  void drawLegend() {

    int rectX, rectY, rectWidth, rectHeight;

    rectX = 10;
    rectY = 460;
    rectWidth = 10;
    rectHeight = 10;

    // draw a legend showing what each color represents
    strokeWeight(1);

    stroke(COLOR_ORIGINAL_DATA);
    fill(COLOR_ORIGINAL_DATA);
    rect(rectX, rectY, rectWidth, rectHeight);
    fill(255);
    text("Original input data", rectX + 20, rectY + 10);

    rectY += dptextSizePlus2;
    stroke(COLOR_KERNEL_DATA);
    fill(COLOR_KERNEL_DATA);
    rect(rectX, rectY, rectWidth, rectHeight);
    fill(255);
    text("Convolution kernel", rectX + 20, rectY + 10);

    rectY += dptextSizePlus2;
    stroke(COLOR_OUTPUT_DATA);
    fill(COLOR_OUTPUT_DATA);
    rect(rectX, rectY, rectWidth, rectHeight);
    fill(255);
    text("Smoothed convolution output data", rectX + 20, rectY + 10);

    rectY += dptextSizePlus2;
    stroke(COLOR_FIRST_DIFFERENCE);
    fill(COLOR_FIRST_DIFFERENCE);
    rect(rectX, rectY, rectWidth, rectHeight);
    fill(255);
    text("1st difference of convolution output data", rectX + 20, rectY + 10);
  }

  void drawGrid(float gWidth, float gHeight, float divisor)
  {
    float widthSpace = gWidth/divisor;   // Number of Vertical Lines
    float heightSpace = gHeight/divisor; // Number of Horozontal Lines
    int i;
    int w;

    strokeWeight(1);
    stroke(25, 25, 25); // White Color

    // Draw vertical
    for (i=0; i < gWidth; i+= widthSpace) {
      line(i, 0, i, gHeight);
    }
    // Draw Horizontal
    for (w=0; w < gHeight; w+= heightSpace) {
      line(0, w, gWidth, w);
    }
  }

  void drawGrid2(float startX, float stopX, float startY, float stopY, float scaleX, float scaleY) {

    float spacingX;
    float spacingY;

    spacingY = scaleY * 256;

    strokeWeight(1);
    spacingX = scaleX * 16;

    stroke(20, 20, 20); // White Color
    for (float x = startX; x <= stopX; x += spacingX) {
      line(x, startY, x, stopY);
    }
    for (float y = startY; y <= stopY; y += spacingY) {
      line(startX, y, stopX, y);
    }
  }

  int grey(color p) {
    return max((p >> 16) & 0xff, (p >> 8) & 0xff, p & 0xff);
  }

  int Pixelbrightness(color p) {

    int r = (p >> 16) & 0xff;
    int g = (p >> 8) & 0xff;
    int b = p & 0xff;
    int value = 299*(r) + 587*(g) + 114*(b);

    if (value > 0) {
      value = value/1000;
    } else {
      value = 0;
    }

    return value;
  }

  color ScaledColorFromInt(int value, int MaxValueRef) {
    return color(map(value, 0, MaxValueRef, 0, 255));
  }

  float LinearInterpolate(float y1, float y2, float mu) {
    return(y1*(1-mu)+y2*mu);
  }

  //double qinty(float ym1, float y0, float yp1) { // another subpixel fit function, not used, but here for reference
  ////QINT - quadratic interpolation of three adjacent samples
  ////[p,y,a] = qint(ym1,y0,yp1) 

  //// returns the extremum location p, height y, and half-curvature a
  //// of a parabolic fit through three points. 
  //// Parabola is given by y(x) = a*(x-p)^2+b, 
  //// where y(-1)=ym1, y(0)=y0, y(1)=yp1. 

  //double p = (yp1 - ym1)/(2*(2*y0 - yp1 - ym1));
  ////double y = y0 - 0.25*(ym1-yp1)*p;
  ////float a = 0.5*(ym1 - 2*y0 + yp1);

  //return p;
  //}

  //int[] FindEdges(){
  //  // a function  for future use perhaps, otherwise ignore.
  //  // Edge finder for use after gaussianLaplacian convolution
  //  // Set kernel to gaussianLaplacian 

  //  // This function is part of the JFeatureLib project: https://github.com/locked-fg/JFeatureLib
  //  // I refactored it for 1d use, (the original is for 2d photos or images) and made other
  //  // changes to make it follow my way of doing things here.

  //  // ZeroCrossing is an algorithm to find the zero crossings in an image 
  //  // original author: Timothy Sharman 

  //  // Find the zero crossings
  //  // If when neighbouring points are multiplied the result is -ve then there 
  //  // must be a change in sign between these two points. 
  //  // If the change is also above the thereshold then set it as a zero crossing.   

  //  // I played with this code for sake of learning, but I don't expect to use it
  //  // unless I can find a sub-pixel method to go with it. 

  //  // Using the 1st derivative is preferable to using this 2nd derivative edge 
  //  // finding method, because this 2nd derivative edge finder works at pixel resolution 
  //  // only; there is not a sub-pixel method for it that I am aware of (yet). Whereas, the 
  //  // peaks present in the first derivative can be prpcessed with various sub-pixel
  //  // routines via many different methods, such as a fitting a parabola to the top 3 
  //  // points (quadratic interpolation), gaussuan estimation, linear regression, etc.

  //  // To find the zero crossings in the image after applying the LOG kernel you must check 
  //  // each point in the array to see if it lies on a zero crossing
  //  // This is done by checking the neighbours around the pixel.

  //  int outerPtrXMinus1 = outerPtrX -1;
  //  int outerPtrXPlus1 = outerPtrX + 1;
  //  int edgeLimit = 10;
  //  int edgeThresh = 64;
  //  boolean edgeLimiter = false;
  //  int[] edges = new int[dpDataLen];

  //  if (outerPtrX > 0 && outerPtrXMinus1 < SENSOR_PIXELS) { 
  //    if(edgeLimiter){ 
  //      edgeThresh = edgeLimit; 
  //    } else { 
  //      edgeThresh = 0; 
  //    }
  //  }

  //  if(output[outerPtrXMinus1]*output[outerPtrXPlus1] < 0){ 
  //    if(Math.abs(output[outerPtrXMinus1]) + Math.abs(output[outerPtrXPlus1]) > edgeThresh){ 
  //       edges[outerPtrXMinus1] = 255;   // white
  //    } else { 
  //      edges[outerPtrXMinus1] = 0;      // black
  //    } 
  //  } else if(output[outerPtrX+1]*output[outerPtrXMinus1] < 0){
  //    if(Math.abs(output[outerPtrXPlus1])+Math.abs(output[outerPtrXMinus1]) > edgeThresh){ 
  //      edges[outerPtrXMinus1] = 255;   // white
  //      } else { 
  //        edges[outerPtrXMinus1] = 0;   // black
  //      } 
  //  } else { 
  //    edges[outerPtrXMinus1] = 0; 
  //  } 

  //return edges;
  //}
}