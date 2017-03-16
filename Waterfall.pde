class Waterfall {
  // Written by Douglas Mayhew, 3/14/2017
  // This class draws a waterfall display

  private PApplet p; // parent object
  // Waterfall variables
  int _wfWidth;
  int _wfHeight;

  int _surfaceWidth;
  int _surfaceHeight;

  int _x;
  int _y;

  int _getPixelIndex;
  int _setPixelIndex;

  float _noiseInput;     // used for generating smooth noise for original data; lower values create smoother noise
  float _noiseIncrement; // the increment of change of the noise input

  //int dataArray[] = new int[0];
  
  // array which feeds the waterfall display
  public int[] waterfallTop = new int[0];
  PImage waterfallImg;
  
  Waterfall (PApplet p) {

    this.p = p;
  }

  void init(int wfWidth, int wfHeight) {
    
    _wfWidth = wfWidth;
    _wfHeight = wfHeight;
    
    waterfallTop = new int[_wfWidth];  // feeds the waterfall display
    // init the waterfall image
    
    waterfallImg = createImage(_wfWidth, _wfHeight, RGB);
    
    // used for generating smooth noise for original data; lower values are smoother noise
    _noiseInput = 0.1;
    // the increment of change of the noise input
    _noiseIncrement = _noiseInput;
  }

  void initNoise(float noiseInput, float noiseIncrement) {
    // used for generating smooth noise for original data; lower values are smoother noise
    _noiseInput = noiseInput;

    // the increment of change of the noise input
    _noiseIncrement = noiseIncrement;
  }
  
  void refresh() {
    waterfallImg.loadPixels();
    // Copy a row of pixels from waterfallTop[] and write them to the top row of the waterfall image
    // scroll all rows down one row to make room for the new one.
    //waterfallTop = perlinNoiseColor(255, wWidth); // perlin noise instead, try uncommenting this line, it looks like tv static

    arrayCopy(waterfallTop, waterfallImg.pixels);

    for (_y = _wfHeight-2; _y > -1; _y--) {            // rows, begin at 0 (the bottom of screen) and count to top -2
      for (_x = 0; _x < _wfWidth; _x++) {              // columns left to right
        _getPixelIndex = (_y * _wfWidth) + _x;      // one pixel
        _setPixelIndex = _getPixelIndex + _wfWidth; // move down one row
        waterfallImg.pixels[_setPixelIndex] = waterfallImg.pixels[_getPixelIndex]; // copy the pixel to the row below
      }
    }

    // We are done incrementing the waterfall. 
    // Erase the old waterfall feeder data so it does not accumulate
    for (_x = 0; _x < _wfWidth; _x++) { 
      waterfallTop[_x] = 0;
    }

    waterfallImg.updatePixels();
    
    // show the waterfall image prior to drawing the subpixel text so the text is on top
    image(waterfallImg, 0, height - _wfHeight, _wfWidth, _wfHeight);
  }

  int[] perlinNoiseColor(int multY, int dataLen) {

    int temp;
    int[] rdOut = new int[dataLen];
    for (int c = 0; c < dataLen; c++) {
      // adjust smoothness with noise input
      _noiseInput = _noiseInput + _noiseIncrement; 
      if (_noiseInput > 10000) {
        _noiseInput = _noiseIncrement;
      }
      // perlin noise
      temp = int(map(noise(_noiseInput), 0, 1, 0, multY));  
      rdOut[c] = color(temp);
      //println (noise(noiseInput));
    }
    return rdOut;
  }
}