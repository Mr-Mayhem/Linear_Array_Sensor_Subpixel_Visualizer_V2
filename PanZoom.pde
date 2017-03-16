/**
 * Pan-Zoom Controller, written by Douglas Mayhew November 20, 2016.
 
 * We do not use Processing's built-in translate() methods to apply 
 * translations, because we don't want to hide that functionality 
 * from the experimenter, and because we want to use values from 
 * this PanZoom object to enable us to ignore working on
 * data that would not appear in the display window anyway.
 
 * Pan-Zoom Controller,
 * Allows to move and scale a drawing using mouse and keyboard. Mouse wheel
 * changes the scale, mouse drag or keyboard arrows change the panning
 * (movement).
 * 
 * @author Bohumir Zamecnik, modified by Doug Mayhew Dec 7, 2016
 * @license MIT
 * 
 * Inspired by "Pan And Zoom" by Dan Thompson, licensed under Creative Commons
 * Attribution-Share Alike 3.0 and GNU GPL license. Work:
 * http://openprocessing.org/visuals/?visualID= 46964
 */

public class PanZoomX {

  private final float DIR_LEFT = -1.0;
  private final float DIR_RIGHT = 1.0;

  private float panVelocity = 40;
  private float scaleVelocity = 0.05;
  private float minLogScale = -4;
  private float maxLogScale = 16;

  private float logScale = 0.1;
  private float scale_x = 1;
  private float scale_y = 0.078;
  private float pan_x = 1;
  private float pan_y = 0;
  private float maxpan_x = 0;
  private float maxpan_y = 0;

  private PApplet p;

  public PanZoomX(PApplet p, float DataLen) {
    this.p = p;
    maxpan_x = DataLen * 8;
    pan_x = (width/2)-(DataLen/2);
  }

  public void pzmouseDragged() {
    int mousex = p.mouseX;
    int pmousex = p.pmouseX;
    pan_x = pan_x + mousex - pmousex;
    pan_x = constrain(pan_x, -maxpan_x * scale_x, (maxpan_x * scale_x) + maxpan_x);


    //int mousey = p.mouseY;
    //int pmousey = p.pmouseY;
    //pan_y = pan_y + mousey - pmousey;
    //constrain(pan_y, -maxpan_y, maxpan_y);
  }

  public void pzkeyPressed() {
    if (p.key == PConstants.CODED) {
      switch (p.keyCode) {
      case PApplet.LEFT:
        moveByKey(DIR_LEFT);
        break;
      case PApplet.RIGHT:
        moveByKey(DIR_RIGHT);
        break;
      }
    }
  }

  public void pzmouseWheel(int step) {
    logScale = constrain(logScale + step * scaleVelocity, minLogScale, maxLogScale);
    float prevScale = scale_x;
    if (logScale != 0) {
      scale_x = (float) Math.pow(2, logScale);
    }

    int mousex = p.mouseX;
    pan_x = mousex + ((pan_x - mousex) * scale_x) / prevScale;
    //pan_x = constrain(pan_x, -maxpan_x * scale_x, (maxpan_x * scale_x) + maxpan_x);
  }

  private void moveByKey(float directionx) {
    pan_x = pan_x + (directionx * panVelocity);
  }

  public float getScaleX() {
    return scale_x;
  }

  public void setScaleX(float scaleX) {
    this.scale_x = scaleX;
  }

  public float getScaleY() {
    return scale_y;
  }

  public void setScaleY(float scaleY) {
    this.scale_y = scaleY;
  }

  public float getPanX() {
    return pan_x;
  }

  public void setPanX(float panX) {
    this.pan_x = panX;
  }

  public float getMaxPanX() {
    return maxpan_x;
  }

  public void setMaxPanX(float maxPan_X) {
    this.maxpan_x = maxPan_X;
  }

  public float getPanY() {
    return pan_y;
  }

  public void setPanY(float panY) {
    this.pan_y = panY;
  }

  public float getMaxPanY() {
    return maxpan_y;
  }

  public void setMaxPanY(float maxPan_Y) {
    this.maxpan_y = maxPan_Y;
  }

  public void setPanVelocity(float panVelocity) {
    this.panVelocity = panVelocity;
  }

  public void setMinLogScale(float minLogScale) {
    if (!(minLogScale < 1)) {
      this.minLogScale = minLogScale;
    }
  }

  public void setMaxLogScale(float maxLogScale) {
    this.maxLogScale = maxLogScale;
  }
}