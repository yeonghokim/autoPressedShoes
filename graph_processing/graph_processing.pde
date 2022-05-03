import processing.serial.*;
import grafica.*;

Serial myPort;  // Create object from Serial class
String val;     // Data received from the serial port
GPointsArray points;
int i ;
GPlot plot;

HScrollbar hs;
void setup()
{
  size(1840, 960);
  background(255);
  
  hs = new HScrollbar(0,height-8,width,16,16);
  
  int nPoints = 100;
  points = new GPointsArray(nPoints);

  for (i = 0; i < nPoints; i++) {
    points.add(i, 10*noise(0.1*i));
  }
  
  plot = new GPlot(this);
  plot.setPos(40, 100);
  plot.setTitleText("Shoes pressing Graph");
  plot.getXAxis().setAxisLabelText("x axis");
  plot.getYAxis().setAxisLabelText("y axis");
  
  plot.setDim(600+i*10, 600);
  plot.setPoints(points);
  
  plot.defaultDraw();
  
  // I know that the first port in the serial list on my mac
  // is Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 9600);
}

void draw()
{
  if ( myPort.available() > 0){  // If data is available,
    val = myPort.readStringUntil('\n');         // read it and store it in val
    println(val); //print it out in the console
  }
  points.add(i, 10*noise(0.1*i));
  i++;
  
  float graphPos = hs.getPos()-width/2;
  plot.setPos(-graphPos, 100);
  
  plot.setDim(600+i*10, 600);
  plot.setPoints(points);
  plot.defaultDraw();
  hs.update();
  hs.display();
  delay(10);
}
class HScrollbar {
  int swidth, sheight;    // width and height of bar
  float xpos, ypos;       // x and y position of bar
  float spos, newspos;    // x position of slider
  float sposMin, sposMax; // max and min values of slider
  int loose;              // how loose/heavy
  boolean over;           // is the mouse over the slider?
  boolean locked;
  float ratio;

  HScrollbar (float xp, float yp, int sw, int sh, int l) {
    swidth = sw;
    sheight = sh;
    int widthtoheight = sw - sh;
    ratio = (float)sw / (float)widthtoheight;
    xpos = xp;
    ypos = yp-sheight/2;
    spos = xpos;
    newspos = spos;
    sposMin = xpos;
    sposMax = xpos + swidth - sheight;
    loose = l;
  }

  void update() {
    if (overEvent()) {
      over = true;
    } else {
      over = false;
    }
    if (mousePressed && over) {
      locked = true;
    }
    if (!mousePressed) {
      locked = false;
    }
    if (locked) {
      newspos = constrain(mouseX-sheight/2, sposMin, sposMax);
    }
    if (abs(newspos - spos) > 1) {
      spos = spos + (newspos-spos)/loose;
    }
    
  }

  float constrain(float val, float minv, float maxv) {
    return min(max(val, minv), maxv);
  }

  boolean overEvent() {
    if (mouseX > xpos && mouseX < xpos+swidth &&
       mouseY > ypos && mouseY < ypos+sheight) {
      return true;
    } else {
      return false;
    }
  }

  void display() {
    noStroke();
    fill(204);
    rect(xpos, ypos, swidth, sheight);
    if (over || locked) {
      fill(0, 0, 0);
    } else {
      fill(102, 102, 102);
    }
    rect(spos, ypos, sheight, sheight);
  }

  float getPos() {
    // Convert spos to be values between
    // 0 and the total width of the scrollbar
    return (spos + swidth/2 - sheight/2) * ratio;
  }
}
