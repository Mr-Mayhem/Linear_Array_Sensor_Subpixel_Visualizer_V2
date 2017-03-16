/* Serial_Thread Class for Linear_Array_Subpixel_Visualizer_V2
 Written by Douglas Mayhew, March 10, 2017
 
 Serial encoding and decoding protocol idea from user Robin2 in forum.arduino.cc:
 https://forum.arduino.cc/index.php?topic=225329.0
 */

import java.util.concurrent.TimeUnit;
import processing.serial.*;

class SerialThread extends Thread
{
  private PApplet parent;       // parent object, used to register shutdown of this thread when main thread quits
  final int START_MARKER = 254; // indicates start of data frame
  final int END_MARKER = 255;   // indicates end of data frame
  final int SPECIAL_BYTE = 253; // indicates an escaped byte value in the encoding scheme to avoid sync byte duplicats;
  // SPECIAL_BYTE converts to the original data value by summing with the byte immediately after it.

  final int MODE_DEBUG = 0;
  final int MODE_SENSOR_PIXELS = 1;
  final int MODE_SHADOW_DATA = 2;
  final int MODE_BYTE = 3;
  final int MODE_INT = 4;
  final int MODE_FLOAT = 5;
  final int MODE_STOP = 6;

  boolean running;        // while loop control variable
  int delayCount;
  int found = -1;
  int loopCount = 0;      // counts how many while loops have occured since last reset.
  int bitCount;           // number of bits we summed during elapsed time in rate calculation
  int dataFrameCount;     // number of sensor data frames summed during elapsed time in rate calculation
  int shadowFrameCount;   // number of shadow info data frames summed during elapsed time in rate calculation
  int bytesRead;          // number of bytes read, controls if we pause the while loop or not; 
  int decodedBytesRead = 0;
  int capturedFrames = 0;
  int firstFrame = 1;
  int bytesAvailableLocal = 0;
  int modeByte = 0;
  int lengthIndicatorByte = 0;
  int framesToCaptureMax = 2;
  int missesSinceLastRead = 0;
  int missesSinceLastReadMaxBeforeExit = 5000;
  int maxDebugMsgLen = 1024;  // max length of debug text message, for array size limit and prevent choking on firehose
  boolean isFirstFrame = false;

  Serial myPort;       // One Serial object, receives serial port data from Teensy 3.6 running sensor driver sketch
  /** Last time in nanoseconds that bitRate was checked */
  protected long bitRateLastNanos = 0;

  SerialThread(PApplet p)
  {
    this.parent = p;
    //parent.registerMethod("dispose", this);
    running = false;
  }

  void start(String serialPortName, int serialBaudRate)
  {
    running = true;
    String portName = Serial.list()[0];
    println("Local Machine Serial Port [0] Name = " + portName);

    // Serial settings at serialReader.start() arguments to this on first tab, the main page.
    myPort = new Serial(this.parent, serialPortName, serialBaudRate); // 12.5 megabits per second Teensy 3.6
    //myPort.clear(); // prevents bad sync glitch from happening, empties buffer on start
    isFirstFrame = true;
    super.start();
  }

  void run()
  {
    while (running)
    {
      try
      {
        found = -1;
        // this reads a chunk of serial data into byteArray[],
        // stored in the main Processing animation thread.
        bytesAvailableLocal = myPort.available();  // This is relatively cheap
        if (bytesAvailableLocal > 4) 
        {  // if there is at least 5 bytes available (4 signal + 1 data) ...
          if (isFirstFrame)
          {
            isFirstFrame = false;
            myPort.clear();
            //bytesAvailableLocal = myPort.available();
            //if (bytesAvailableLocal > byteArray.length) {
            //  byteArray = new byte[bytesAvailableLocal];
            //}
          }
          bytesRead = myPort.readBytesUntil(END_MARKER, byteArray); // copy serial data into byteArray[]
          if (bytesRead > 4) 
          { // better safe than sorry. If there is at least 5 bytes read...
            if ((byteArray[0] & 0xFF) == START_MARKER) 
            { // if START_MARKER is present at index 0, we have a complete frame
              found = 1;
              missesSinceLastRead = 0;
              bitCount += (bytesRead * 10);  // accumulate bitCount
              modeByte = (byteArray[1] & 0xFF); // modeByte tells us the data format to expect
              lengthIndicatorByte = (byteArray[2] & 0xFF);   // unused byte is left over from modulo 2 arduino int to byte loop

              if (capturedFrames < framesToCaptureMax) 
              {  // for troubleshooting, show the first complete data frame and the decoded one too
                printEncodedRXData(bytesRead); // print the first complete frame data to the Processing debug window
                switch (modeByte) {
                case MODE_DEBUG:
                  decodedBytesRead = decodeDebugMsg(bytesRead);  // decodes bytes from byteArray[] into msgArrayDecoded[]
                  println("**** decodedBytesRead = " + decodedBytesRead);
                  printDecodedDebugMsg(decodedBytesRead);
                  break;
                case MODE_SENSOR_PIXELS:
                  dataFrameCount++;
                  decodedBytesRead = decodePixelData(bytesRead);  // decodes bytes from byteArray[] into byteArrayDecoded[]
                  println("**** decodedBytesRead = " + decodedBytesRead);
                  printDecodedSensorData(decodedBytesRead);
                  if (drawHasFinished) redraw();
                  break;
                case MODE_SHADOW_DATA:
                  shadowFrameCount++;
                  decodedBytesRead = decodeShadowData(bytesRead);  // decodes bytes from byteArray[] into shadowArrayDecoded[]
                  println("**** decodedBytesRead = " + decodedBytesRead);
                  printDecodedSensorShadowData(decodedBytesRead);
                  if (drawHasFinished) redraw();
                  break;
                  //case MODE_BYTE: // for demo code purposes, I don't use it
                  //  decodedBytesRead = decodeVariable(bytesRead);  // decodes bytes from byteArray[] into varArrayDecoded[]
                  //  println("**** decodedBytesRead = " + decodedBytesRead);
                  //  printDecodedByte(decodedBytesRead);
                  //  break;
                  //case MODE_INT: // for demo code purposes, I don't use it
                  //  decodedBytesRead = decodeVariable(bytesRead);  // decodes bytes from byteArray[] into varArrayDecoded[]
                  //  println("**** DdecodedBytesRead = " + decodedBytesRead);
                  //  printDecodedInteger(decodedBytesRead);
                  //  break;
                  //case MODE_FLOAT: // for demo code purposes, I don't use it
                  //  decodedBytesRead = decodeVariable(bytesRead);  // decodes bytes from byteArray[] into varArrayDecoded[]
                  //  println("**** decodedBytesRead = " + decodedBytesRead);
                  //  printDecodedFloat(decodedBytesRead);
                  //  break;
                  //case MODE_STOP: // for demo code purposes, I don't use it
                  //  println("Serial Parsing Error, MODE_STOP: " + modeByte);
                  //  println("**** decodedBytesRead = 0");
                  //  break;
                default:
                  println("Serial Parsing Error, unknown modeByte: " + modeByte);
                  println("**** decodedBytesRead = 0");
                }
                capturedFrames++;
              } else {
                switch (modeByte) {
                case MODE_DEBUG:
                  decodedBytesRead = decodeDebugMsg(bytesRead);  // decodes bytes from byteArray[] into msgArrayDecoded[]
                  //println("**** decodedBytesRead = " + decodedBytesRead);
                  //printDecodedDebugMsg(decodedBytesRead);
                  break;
                case MODE_SENSOR_PIXELS:
                  dataFrameCount++;
                  decodedBytesRead = decodePixelData(bytesRead);  // decodes bytes from byteArray[] into byteArrayDecoded[]
                  if (drawHasFinished) redraw();
                  break;
                case MODE_SHADOW_DATA:
                  shadowFrameCount++;
                  decodedBytesRead = decodeShadowData(bytesRead);  // decodes bytes from byteArray[] into shadowArrayDecoded[]
                  drawDecodedSensorShadowData(decodedBytesRead);
                  if (drawHasFinished) redraw();
                  break;
                  //case MODE_BYTE: // for demo code purposes, I don't use it
                  //  decodedBytesRead = decodeVariable(bytesRead);  // decodes bytes from byteArray[] into varArrayDecoded[]

                  //  break;
                  //case MODE_INT: // for demo code purposes, I don't use it
                  //  decodedBytesRead = decodeVariable(bytesRead);  // decodes bytes from byteArray[] into varArrayDecoded[]

                  //  break;
                  //case MODE_FLOAT: // for demo code purposes, I don't use it
                  //  decodedBytesRead = decodeVariable(bytesRead);  // decodes bytes from byteArray[] into varArrayDecoded[]

                  //  break;
                  //case MODE_STOP: // for demo code purposes, I don't use it
                  //  println("Serial Parsing Error, MODE_STOP: " + modeByte);
                  //  println("**** decodedBytesRead = 0");
                  //  break;
                default:
                  println("Serial Parsing Error, unknown modeByte: " + modeByte);
                  println("**** decodedBytesRead = 0");
                }
              }
            }
          }
        }
        if (found == -1) {
          missesSinceLastRead++;
          if (missesSinceLastRead > 5) 
          {
            Thread.sleep(0, 100); // if we received no data frame above, we pause the while loop a moment
            // so as to not waste as many cpu cycles polling it.
          }
          if (missesSinceLastRead > missesSinceLastReadMaxBeforeExit) 
          {
            println("Quitting Serial_Thread and Processing due to serial port data interruption");
            myPort.stop();
            quit();
            exit();
          }
        }
        if (loopCount % 1000 == 0) { // every 1000 loops
          bytesAvailable = bytesAvailableLocal; // copy local to global for use on the screen
          calcRatesPerSecond();
          loopCount = 0;
        }
        loopCount++;
      }
      catch(Exception e)
      {
        println(e);
      }
    }
  }

  void calcRatesPerSecond() {
    // calculate bits per millisecond
    long now = System.nanoTime();
    double elapsedTime = (now - bitRateLastNanos) / 1000000000.0;
    //println("elapsedTime = " + elapsedTime + " missesSinceLastRead = " + missesSinceLastRead);

    // bits received per time unit
    float instantaneousBitRate = (float) (bitCount / elapsedTime);  

    // sensor data frames received per time unit
    float instantaneousDataFrameRate = (float) (dataFrameCount / elapsedTime);   

    // shadow info data frames received per time unit
    float instantaneousShadowFrameRate = (float) (shadowFrameCount / elapsedTime);   

    bitRate = (bitRate * 0.8f) + (instantaneousBitRate * 0.2f); // low pass filter
    bitCount = 0;

    dataFrameRate = (dataFrameRate * 0.8f) + (instantaneousDataFrameRate * 0.2f); // low pass filter
    dataFrameCount = 0;

    shadowFrameRate = (shadowFrameRate * 0.8f) + (instantaneousShadowFrameRate * 0.2f); // low pass filter
    shadowFrameCount = 0;

    bitRateLastNanos = now;  // set it up for the next frame
  }

  int decodeDebugMsg(int encodedLength) {
    int bytesRXCount = 0;
    int n;
    int x;

    // 3 skips start, mode, and length bytes, -1 omits the end marker
    for (n = 3; n < encodedLength - 1; n++) { 
      x = byteArray[n] & 0xFF;
      if (x == SPECIAL_BYTE) { // byte 253, encoded as two bytes, 253 + n
        n++; // next byte
        x = x + byteArray[n] & 0xFF; // add the value of the special byte 
        // to the value of the byte which follows it,
        // to decode the original byte value
      }
      msgArrayDecoded[bytesRXCount] = (byte) (x & 0xFF); // copy the decoded byte pair to dest array
      bytesRXCount ++; // maintain a count so we know how many bytes were received after decoding
    }
    return bytesRXCount;
  }

  int decodePixelData(int encodedLength) {
    int bytesRXCount = 0;
    int n;
    int x;

    // 3 skips start, mode, and length bytes, -1 omits the end marker
    for (n = 3; n < encodedLength - 1; n++) { 
      x = byteArray[n] & 0xFF;
      if (x == SPECIAL_BYTE) { // byte 253, encoded as two bytes, 253 + n
        n++; // next byte
        x = x + byteArray[n] & 0xFF; // add the value of the special byte 
        // to the value of the byte which follows it,
        // to decode the original byte value
      }
      byteArrayDecoded[bytesRXCount] = (byte) (x & 0xFF); // copy the decoded byte pair to dest array
      bytesRXCount ++; // maintain a count so we know how many bytes were received after decoding
    }
    return bytesRXCount;
  }

  int decodeShadowData(int encodedLength) {
    int bytesRXCount = 0;
    int n;
    int x;

    // 3 skips start, mode, and length bytes, -1 omits the end marker
    for (n = 3; n < encodedLength - 1; n++) { 
      x = byteArray[n] & 0xFF;
      if (x == SPECIAL_BYTE) { // byte 253, encoded as two bytes, 253 + n
        n++; // next byte
        x = x + byteArray[n] & 0xFF; // add the value of the special byte 
        // to the value of the byte which follows it,
        // to decode the original byte value
      }
      shadowArrayDecoded[bytesRXCount] = (byte) (x & 0xFF); // copy the decoded byte pair to dest array
      bytesRXCount ++; // maintain a count so we know how many bytes were received after decoding
    }
    return bytesRXCount;
  }

  int decodeVariable(int encodedLength) {
    int bytesRXCount = 0;
    int n;
    int x;

    // 3 skips start, mode, and length bytes, -1 omits the end marker
    for (n = 3; n < encodedLength - 1; n++) { 
      x = byteArray[n] & 0xFF;
      if (x == SPECIAL_BYTE) { // byte 253, encoded as two bytes, 253 + n
        n++; // next byte
        x = x + byteArray[n] & 0xFF; // add the value of the special byte 
        // to the value of the byte which follows it,
        // to decode the original byte value
      }
      varArrayDecoded[bytesRXCount] = (byte) (x & 0xFF); // copy the decoded byte pair to dest array
      bytesRXCount ++; // maintain a count so we know how many bytes were received after decoding
    }
    return bytesRXCount;
  }

  void printEncodedRXData(int encodedLength) {
    int i;
    int rxByte;

    println("");
    println("*** Raw encoded RX Data, captured frame # " + capturedFrames + " encodedLength = " + encodedLength);
    println("N_BYTES_ENCODED_FRAME = " + N_BYTES_ENCODED_FRAME);
    println("bytesAvailableLocal = " + bytesAvailableLocal);
    println("bytesRead = " + bytesRead);
    println("modeByte = " + modeByte);
    println("lengthIndicatorByte = " + lengthIndicatorByte);

    for (i = 0; i < encodedLength; i++) {
      rxByte = byteArray[i] & 0xFF;

      if (rxByte == START_MARKER) {
        println("byteArray[" + i + "] = " + rxByte + " = START_MARKER");
      } else if (rxByte == END_MARKER) {
        println("byteArray[" + i + "] = " + rxByte + " = END_MARKER");
      } else if (rxByte == SPECIAL_BYTE) {
        println("byteArray[" + i + "] = " + rxByte + " = SPECIAL_BYTE Decoded Value = " + (rxByte + byteArray[i] & 0xFF));
      } else if (i == 1) {
        println("byteArray[" + i + "] = " + rxByte + " (modeByte)");
      } else if (i == 2) {
        println("byteArray[" + i + "] = " + rxByte + " (lengthIndicatorByte)");
      } else {
        println("byteArray[" + i + "] = " + rxByte);
      }
    }
    println("");
  }

  void printDecodedDebugMsg(int decodedLength) {
    int i;
    int rxByte = 0;

    for (i = 0; i < decodedLength; i++) {
      rxByte = msgArrayDecoded[i] & 0xFF;
      println("msgArrayDecoded[" + i + "] = " + rxByte + "[" + char(rxByte) + "]"); // print byte only
    }
    String ardunoDebugStr = new String(msgArrayDecoded);
    println("Arduino Debug Message: " + ardunoDebugStr);
  }

  void printDecodedSensorData(int decodedLength) {
    int i;
    int rxByte = 0;
    int intValue = 0;

    for (i = 0; i < decodedLength; i++) {
      rxByte = byteArrayDecoded[i] & 0xFF;
      if ((i & 1) == 0) { // if i is an even number
        intValue = (byteArrayDecoded[i]<< 8 | (byteArrayDecoded[i+1] & 0xFF)); //get the value 
        println("byteArrayDecoded[" + i + "] = " + rxByte + " Value: " + intValue); // print byte and integer vals
      } else {
        println("byteArrayDecoded[" + i + "] = " + rxByte); // print byte only
      }
    }
  }

  void drawDecodedSensorShadowData(int decodedLength) {
    int intValue1 = 0;
    int intValue2 = 0;
    float floatValue1;
    float floatValue2;

    if (decodedLength == 8) {
      intValue1 = (shadowArrayDecoded[0] & 0xFF) // convert 4 bytes to one float
        | ((shadowArrayDecoded[1] & 0xFF) << 8) 
        | ((shadowArrayDecoded[2] & 0xFF) << 16) 
        | ((shadowArrayDecoded[3] & 0xFF) << 24);
      floatValue1 = Float.intBitsToFloat(intValue1);

      intValue2 = (shadowArrayDecoded[4] & 0xFF) // convert 4 bytes to one float
        | ((shadowArrayDecoded[5] & 0xFF) << 8) 
        | ((shadowArrayDecoded[6] & 0xFF) << 16) 
        | ((shadowArrayDecoded[7] & 0xFF) << 24);
      floatValue2 = Float.intBitsToFloat(intValue2);

      subpixelWidth = floatValue1;
      subpixelCenter = floatValue2;

      //subpixelWidthLP = (subpixelWidthLP * 0.9) + (subpixelWidth * 0.1); // a simple lowpass filter
      //subpixelCenterLP = (subpixelCenterLP * 0.9) + (subpixelCenter * 0.1); // a simple lowpass filter
    }
  }

  void printDecodedSensorShadowData(int decodedLength) {
    int i;
    int rxByte = 0;
    int intValue1 = 0;
    int intValue2 = 0;
    float floatValue1;
    float floatValue2;

    if (decodedLength == 8) {
      for (i = 0; i < decodedLength; i++) { // for each decoded byte
        rxByte = shadowArrayDecoded[i] & 0xFF;
        println("shadowArrayDecoded[" + i + "] = " + rxByte); // print each decoded byte that makes up the float
      }

      intValue1 = (shadowArrayDecoded[0] & 0xFF) // convert 4 bytes to one float
        | ((shadowArrayDecoded[1] & 0xFF) << 8) 
        | ((shadowArrayDecoded[2] & 0xFF) << 16) 
        | ((shadowArrayDecoded[3] & 0xFF) << 24);
      floatValue1 = Float.intBitsToFloat(intValue1);

      intValue2 = (shadowArrayDecoded[4] & 0xFF) // convert 4 bytes to one float
        | ((shadowArrayDecoded[5] & 0xFF) << 8) 
        | ((shadowArrayDecoded[6] & 0xFF) << 16) 
        | ((shadowArrayDecoded[7] & 0xFF) << 24);
      floatValue2 = Float.intBitsToFloat(intValue2);

      println("received Sensor Shadow Data: " + floatValue1);
      println("value1: " + floatValue1);
      println("value2: " + floatValue2);

      subpixelWidth = floatValue1;
      subpixelCenter = floatValue2;

      subpixelWidthLP = (subpixelWidthLP * 0.9) + (subpixelWidth * 0.1); // a simple lowpass filter
      subpixelCenterLP = (subpixelCenterLP * 0.9) + (subpixelCenter * 0.1); // a simple lowpass filter
    } else {
      println("Error in printDecodedSensorShadowData: Expected 8 bytes, got " + decodedLength + " bytes");
    }
  }

  void printDecodedByte(int decodedLength) {
    int rxByte = 0;

    if (decodedLength == 1) {
      rxByte = varArrayDecoded[0] & 0xFF; // only one byte
      println("received single byte value: " + rxByte);
    } else {
      println("Error in printDecodedByte: Expected 1 byte, got " + decodedLength + " bytes");
    }
  }

  void printDecodedInteger(int decodedLength) {
    int i;
    int rxByte = 0;
    int intValue = 0;

    if (decodedLength == 4) {
      for (i = 0; i < decodedLength; i++) {  // for each decoded byte
        rxByte = varArrayDecoded[i] & 0xFF;
        println("varArrayDecoded[" + i + "] = " + rxByte); // print each decoded byte that makes up the integer
      }

      intValue = ((varArrayDecoded[0] & 0xFF) << 24)  // convert 4 bytes to one integer
        | ((varArrayDecoded[1] & 0xFF) << 16)
        | ((varArrayDecoded[2] & 0xFF) << 8) 
        | (varArrayDecoded[3] & 0xFF);
      println("received single integer value: " + intValue);
    } else {
      println("Error in printDecodedInteger: Expected 4 bytes, got " + decodedLength + " bytes");
    }
  }

  void printDecodedFloat(int decodedLength) {
    int i;
    int rxByte = 0;
    int intValue = 0;
    float floatValue;

    if (decodedLength == 4) {
      for (i = 0; i < decodedLength; i++) { // for each decoded byte
        rxByte = varArrayDecoded[i] & 0xFF;
        println("varArrayDecoded[" + i + "] = " + rxByte); // print each decoded byte that makes up the float
      }

      intValue = (varArrayDecoded[0] & 0xFF) // convert 4 bytes to one float
        | ((varArrayDecoded[1] & 0xFF) << 8) 
        | ((varArrayDecoded[2] & 0xFF) << 16) 
        | ((varArrayDecoded[3] & 0xFF) << 24);
      floatValue = Float.intBitsToFloat(intValue);
      println("received single float value: " + floatValue);
    } else {
      println("Error in printDecodedFloat: Expected 4 bytes, got " + decodedLength + " bytes");
    }
  }

  public void quit()
  {
    running = false;
    interrupt();
  }

  //public void dispose() {
  //  running = false;
  //  stop();
  //}
}