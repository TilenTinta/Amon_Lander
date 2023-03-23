/// Amon Ground Controll softtware
//  Tinta T.
//  October 2022
//  Amon Project
///

// Librarys
import controlP5.*; // buttons on screen
import processing.serial.*; // Serial communication

ControlP5 cp5;
PImage logo;
PFont nasa;
PFont txtDef;

// Variables for lander
int amon_status = 0;
float main_bat = 0;
float EDF_bat = 0;
float temp = 0;
float press = 0;
float hum = 0;
int connection_ok = 0;
float tvc_xp = 0;
float tvc_xn = 0;
float tvc_yp = 0;
float tvc_yn = 0;

boolean btnOver;
MenuWindow WindowA;

Serial myPort;
String[] ports;
boolean initWindowOpen = false;


// Variables for Graphs
int Graph_width = 450;     // width of graph
int Graph_height = 300;     // height of graph
int[] EDFData = new int[Graph_width];    // data buffer EDF
int[] Altitude = new int[Graph_width];    // data buffer Altitude
int[] xy_velocity = new int[Graph_width];    // data buffer X-Y Velocity
int[] xy_position = new int[Graph_width];    // data buffer X-Y Velocity
int[] orient_x = new int[Graph_width];    // data buffer Orientation X
int[] orient_y = new int[Graph_width];    // data buffer Orientation Y
int[] orient_z = new int[Graph_width];    // data buffer Orientation Z
int[] accel_x = new int[Graph_width];    // data buffer Acceleration X
int[] accel_y = new int[Graph_width];    // data buffer Acceleration Y
int[] accel_z = new int[Graph_width];    // data buffer Acceleration Z

int testData = 0; // brisi
int testDataNeg = 0; // brisi
int updn = 0;


void setup()
{
  frameRate(100);
  smooth();
  size(1920, 1000); // Fullscreen
  logo = loadImage("AMON_logo.png");
  nasa = createFont("nasalization-rg.otf", 20);
  txtDef = createFont("AndaleMono.vlw", 20);
  
  // serial
  ports = Serial.list();
  if (ports.length > 0){
    myPort = new Serial(this, ports[0], 112500);
  };

}

// Decoding recieved packets from lander
void Decode_rx()
{
};

// Decoding packets for transmiting to lander
void Decode_tx()
{
};

// Drawing graph for showing values in percents
void Graph_percent_show(int x_coord, int y_coord, String graph_name) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Base square
  stroke(0, 0, 0);
  fill(150, 150, 150);
  rect(x_coord, y_coord, Graph_width, Graph_height, 5); // 380,230

  // graph label, underline...
  fill(255, 255, 255);
  textSize(20);
  text(graph_name, x_coord + 150, y_coord + 25);
  line(x_coord + 20, y_coord + 30, x_coord + 420, y_coord + 30);

  pushMatrix();
  rotate(radians(270));
  textSize(15);
  translate(-x_coord-100, -y_coord-100);
  text("%", x_coord + 0, y_coord + 0);
  popMatrix();

  pushMatrix();
  rotate(radians(0));
  textSize(15);
  text("sec", x_coord + 200, y_coord + 285);
  popMatrix();

  stroke(0, 0, 0);
  strokeWeight(1);
  line(x_coord + 20, y_coord + 45, x_coord + 20, y_coord + 280);           // y-axis line
  line(x_coord + 12, y_coord + 270, x_coord + 420, y_coord + 270);         // x-axis line

  // Lines for values
  textSize(8);
  text("100", x_coord + 2, y_coord + 73);
  line(x_coord + 18, y_coord + 70, x_coord + 22, y_coord + 70);
  line(x_coord + 18, y_coord + 90, x_coord + 22, y_coord + 90);
  line(x_coord + 18, y_coord + 110, x_coord + 22, y_coord + 110);
  line(x_coord + 18, y_coord + 130, x_coord + 22, y_coord + 130);
  line(x_coord + 18, y_coord + 150, x_coord + 22, y_coord + 150);
  line(x_coord + 18, y_coord + 170, x_coord + 22, y_coord + 170);
  line(x_coord + 18, y_coord + 190, x_coord + 22, y_coord + 190);
  line(x_coord + 18, y_coord + 210, x_coord + 22, y_coord + 210);
  line(x_coord + 18, y_coord + 230, x_coord + 22, y_coord + 230);
  line(x_coord + 18, y_coord + 250, x_coord + 22, y_coord + 250);
  text("0", x_coord + 10, y_coord + 268);
};


// Drawing graph for showing values in meters
void Graph_meter_show(int x_coord, int y_coord, String graph_name) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Base square
  stroke(0, 0, 0);
  fill(150, 150, 150);
  rect(x_coord, y_coord, Graph_width, Graph_height, 5); // 380,230

  // graph label, underline...
  fill(255, 255, 255);
  textSize(20);
  text(graph_name, x_coord + 150, y_coord + 25);
  line(x_coord + 20, y_coord + 30, x_coord + 420, y_coord + 30);

  pushMatrix();
  rotate(radians(270));
  textSize(15);
  translate(-900, -627);
  text("m", x_coord + 0, y_coord + 0);
  popMatrix();

  pushMatrix();
  rotate(radians(0));
  textSize(15);
  text("sec", x_coord + 200, y_coord + 285);
  popMatrix();

  stroke(0, 0, 0);
  strokeWeight(1);
  line(x_coord + 20, y_coord + 45, x_coord + 20, y_coord + 280);           // y-axis line
  line(x_coord + 12, y_coord + 270, x_coord + 420, y_coord + 270);         // x-axis line

  // Lines for values
  textSize(8);
  text("10", x_coord + 8, y_coord + 73);
  line(x_coord + 18, y_coord + 70, x_coord + 22, y_coord + 70);
  line(x_coord + 18, y_coord + 90, x_coord + 22, y_coord + 90);
  line(x_coord + 18, y_coord + 110, x_coord + 22, y_coord + 110);
  line(x_coord + 18, y_coord + 130, x_coord + 22, y_coord + 130);
  line(x_coord + 18, y_coord + 150, x_coord + 22, y_coord + 150);
  line(x_coord + 18, y_coord + 170, x_coord + 22, y_coord + 170);
  line(x_coord + 18, y_coord + 190, x_coord + 22, y_coord + 190);
  line(x_coord + 18, y_coord + 210, x_coord + 22, y_coord + 210);
  line(x_coord + 18, y_coord + 230, x_coord + 22, y_coord + 230);
  line(x_coord + 18, y_coord + 250, x_coord + 22, y_coord + 250);
  text("0", x_coord + 10, y_coord + 268);
};


// Drawing graph for showing values in meters
void Graph_meter_center_show(int x_coord, int y_coord, String graph_name) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Base square
  stroke(0, 0, 0);
  fill(150, 150, 150);
  rect(x_coord, y_coord, Graph_width, Graph_height, 5); // 380,230

  // graph label, underline...
  fill(255, 255, 255);
  textSize(20);
  text(graph_name, x_coord + 150, y_coord + 25);
  line(x_coord + 20, y_coord + 30, x_coord + 420, y_coord + 30);

  pushMatrix();
  rotate(radians(270));
  textSize(15);
  translate(-900, -627);
  text("m", x_coord + 0, y_coord + 0);
  popMatrix();

  pushMatrix();
  rotate(radians(0));
  textSize(15);
  text("sec", x_coord + 200, y_coord + 285);
  popMatrix();

  stroke(0, 0, 0);
  strokeWeight(1);
  line(x_coord + 20, y_coord + 45, x_coord + 20, y_coord + 280);           // y-axis line
  line(x_coord + 12, y_coord + 270, x_coord + 420, y_coord + 270);         // x-axis line

  // Lines for values
  textSize(8);
  text("10", x_coord + 8, y_coord + 73);
  line(x_coord + 18, y_coord + 70, x_coord + 22, y_coord + 70);
  line(x_coord + 18, y_coord + 90, x_coord + 22, y_coord + 90);
  line(x_coord + 18, y_coord + 110, x_coord + 22, y_coord + 110);
  line(x_coord + 18, y_coord + 130, x_coord + 22, y_coord + 130);
  line(x_coord + 18, y_coord + 150, x_coord + 22, y_coord + 150);
  line(x_coord + 18, y_coord + 170, x_coord + 22, y_coord + 170);
  text("0", x_coord + 3, y_coord + 173);
  line(x_coord + 18, y_coord + 190, x_coord + 22, y_coord + 190);
  line(x_coord + 18, y_coord + 210, x_coord + 22, y_coord + 210);
  line(x_coord + 18, y_coord + 230, x_coord + 22, y_coord + 230);
  line(x_coord + 18, y_coord + 250, x_coord + 22, y_coord + 250);
  text("-10", x_coord + 4, y_coord + 268);
};


// Drawing graph for showing values in meters per second
void Graph_meter_sec_show(int x_coord, int y_coord, String graph_name) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Base square
  stroke(0, 0, 0);
  fill(150, 150, 150);
  rect(x_coord, y_coord, Graph_width, Graph_height, 5); // 380,230

  // graph label, underline...
  fill(255, 255, 255);
  textSize(20);
  text(graph_name, x_coord + 150, y_coord + 25);
  line(x_coord + 20, y_coord + 30, x_coord + 420, y_coord + 30);

  pushMatrix();
  rotate(radians(270));
  textSize(15);
  translate(-900, -627);
  text("m/s", x_coord + 0, y_coord + 0);
  popMatrix();

  pushMatrix();
  rotate(radians(0));
  textSize(15);
  text("sec", x_coord + 200, y_coord + 285);
  popMatrix();

  stroke(0, 0, 0);
  strokeWeight(1);
  line(x_coord + 20, y_coord + 45, x_coord + 20, y_coord + 280);           // y-axis line
  line(x_coord + 12, y_coord + 270, x_coord + 420, y_coord + 270);         // x-axis line

  // Lines for values
  textSize(8);
  text("10", x_coord + 8, y_coord + 73);
  line(x_coord + 18, y_coord + 70, x_coord + 22, y_coord + 70);
  line(x_coord + 18, y_coord + 90, x_coord + 22, y_coord + 90);
  line(x_coord + 18, y_coord + 110, x_coord + 22, y_coord + 110);
  line(x_coord + 18, y_coord + 130, x_coord + 22, y_coord + 130);
  line(x_coord + 18, y_coord + 150, x_coord + 22, y_coord + 150);
  line(x_coord + 18, y_coord + 170, x_coord + 22, y_coord + 170);
  line(x_coord + 18, y_coord + 190, x_coord + 22, y_coord + 190);
  line(x_coord + 18, y_coord + 210, x_coord + 22, y_coord + 210);
  line(x_coord + 18, y_coord + 230, x_coord + 22, y_coord + 230);
  line(x_coord + 18, y_coord + 250, x_coord + 22, y_coord + 250);
  text("0", x_coord + 10, y_coord + 268);
};


// Drawing graph for showing values in meters per second squared
void Graph_meter_meter_sec_show(int x_coord, int y_coord, String graph_name) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Base square
  stroke(0, 0, 0);
  fill(150, 150, 150);
  rect(x_coord, y_coord, Graph_width, Graph_height, 5); // 380,230

  // graph label, underline...
  fill(255, 255, 255);
  textSize(20);
  text(graph_name, x_coord + 150, y_coord + 25);
  line(x_coord + 20, y_coord + 30, x_coord + 420, y_coord + 30);

  pushMatrix();
  rotate(radians(270));
  textSize(15);
  translate(-1050, -1050); // -900, -627  1450, 680
  text("m/s^2", 0, 0);
  popMatrix();

  pushMatrix();
  rotate(radians(0));
  textSize(15);
  text("sec", x_coord + 200, y_coord + 285);
  popMatrix();

  stroke(0, 0, 0);
  strokeWeight(1);
  line(x_coord + 20, y_coord + 45, x_coord + 20, y_coord + 280);           // y-axis line
  line(x_coord + 12, y_coord + 270, x_coord + 420, y_coord + 270);         // x-axis line

  // Lines for values
  textSize(8);
  text("6", x_coord + 6, y_coord + 73);
  line(x_coord + 18, y_coord + 70, x_coord + 22, y_coord + 70);
  line(x_coord + 18, y_coord + 90, x_coord + 22, y_coord + 90);
  line(x_coord + 18, y_coord + 110, x_coord + 22, y_coord + 110);
  line(x_coord + 18, y_coord + 130, x_coord + 22, y_coord + 130);
  line(x_coord + 18, y_coord + 150, x_coord + 22, y_coord + 150);
  line(x_coord + 18, y_coord + 170, x_coord + 22, y_coord + 170);
  line(x_coord + 18, y_coord + 190, x_coord + 22, y_coord + 190);
  line(x_coord + 18, y_coord + 210, x_coord + 22, y_coord + 210);
  line(x_coord + 18, y_coord + 230, x_coord + 22, y_coord + 230);
  text("0", x_coord + 6, y_coord + 233);
  line(x_coord + 18, y_coord + 250, x_coord + 22, y_coord + 250);
  text("-4", x_coord + 6, y_coord + 268);
};


// Drawing graph for showing values in degres
void Graph_degres_show(int x_coord, int y_coord, String graph_name) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Base square
  stroke(0, 0, 0);
  fill(150, 150, 150);
  rect(x_coord, y_coord, Graph_width, Graph_height, 5); // 380,230

  // graph label, underline...
  fill(255, 255, 255);
  textSize(20);
  text(graph_name, x_coord + 150, y_coord + 25);
  line(x_coord + 20, y_coord + 30, x_coord + 420, y_coord + 30);

  pushMatrix();
  rotate(radians(270));
  textSize(15);
  translate(-900, -627);
  text("deg", x_coord + 0, y_coord + 0);
  popMatrix();

  pushMatrix();
  rotate(radians(0));
  textSize(15);
  text("sec", x_coord + 200, y_coord + 285);
  popMatrix();

  stroke(0, 0, 0);
  strokeWeight(1);
  line(x_coord + 20, y_coord + 45, x_coord + 20, y_coord + 280);           // y-axis line
  line(x_coord + 12, y_coord + 270, x_coord + 420, y_coord + 270);         // x-axis line

  // Lines for values
  textSize(8);
  text("90", x_coord + 4, y_coord + 73);
  line(x_coord + 18, y_coord + 70, x_coord + 22, y_coord + 70);
  line(x_coord + 18, y_coord + 90, x_coord + 22, y_coord + 90);
  line(x_coord + 18, y_coord + 110, x_coord + 22, y_coord + 110);
  line(x_coord + 18, y_coord + 130, x_coord + 22, y_coord + 130);
  line(x_coord + 18, y_coord + 150, x_coord + 22, y_coord + 150);
  line(x_coord + 18, y_coord + 170, x_coord + 22, y_coord + 170);
  text("0", x_coord + 8, y_coord + 173);
  line(x_coord + 18, y_coord + 190, x_coord + 22, y_coord + 190);
  line(x_coord + 18, y_coord + 210, x_coord + 22, y_coord + 210);
  line(x_coord + 18, y_coord + 230, x_coord + 22, y_coord + 230);
  line(x_coord + 18, y_coord + 250, x_coord + 22, y_coord + 250);
  text("-90", x_coord + 4, y_coord + 268);
};


//************* Drawing graph - line *************//
// Percent
void Graph_percent_data(int x_coord, int y_coord, int data) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Data for drawing graph calculated to percents
  int max_data = 100;
  int draw_data = (y_coord + 270 - ((230 / max_data) * data)) - 1;

  EDFData[Graph_width-1] = draw_data;

  // Shifting data in buffer
  for (int i = 0; i < Graph_width-1; i++)
  {
    EDFData[i] = EDFData[i+1];
  }

  // Data line (y)
  strokeWeight(1);
  for (int i = Graph_width-40; i > 20; i--)
  {
    if (EDFData[i-1] > y_coord + 35)
    {
      stroke(255, 0, 0);
      line(i+x_coord, EDFData[i-1], (i+1)+x_coord, EDFData[i]);
    };
  }
};

// Altitude
void Graph_altitude_data(int x_coord, int y_coord, float data) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Data for drawing graph calculated to percents
  int max_data = 100;
  int draw_data = int (y_coord + 270 - ((230 / max_data) * data)) - 1;

  Altitude[Graph_width-1] = draw_data;

  // Shifting data in buffer
  for (int i = 0; i < Graph_width-1; i++)
  {
    Altitude[i] = Altitude[i+1];
  }

  // Data line (y)
  strokeWeight(1);
  for (int i = Graph_width-40; i > 20; i--)
  {
    if (Altitude[i-1] > y_coord + 35)
    {
      stroke(255, 0, 0);
      line(i+x_coord, Altitude[i-1], (i+1)+x_coord, Altitude[i]);
    };
  }
};

// Velocity
void Graph_velocity_data(int x_coord, int y_coord, float data) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Data for drawing graph calculated to percents
  int max_data = 100;
  int draw_data = int (y_coord + 270 - ((230 / max_data) * data)) - 1;

  Altitude[Graph_width-1] = draw_data;

  // Shifting data in buffer
  for (int i = 0; i < Graph_width-1; i++)
  {
    Altitude[i] = Altitude[i+1];
  }

  // Data line (y)
  strokeWeight(1);
  for (int i = Graph_width-40; i > 20; i--)
  {
    if (Altitude[i-1] > y_coord + 35)
    {
      stroke(255, 0, 0);
      line(i+x_coord, Altitude[i-1], (i+1)+x_coord, Altitude[i]);
    };
  }
};

// Acceleration X
void Graph_accel_x_data(int x_coord, int y_coord, float data) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Data for drawing graph calculated to percents
  int max_data = 100;
  int draw_data = int (y_coord + 230 - ((190 / max_data) * data)) - 1;

  accel_x[Graph_width-1] = draw_data;

  // Shifting data in buffer
  for (int i = 0; i < Graph_width-1; i++)
  {
    accel_x[i] = accel_x[i+1];
  }

  // Data line (y)
  strokeWeight(1);
  for (int i = Graph_width-40; i > 20; i--)
  {
    if (accel_x[i-1] > y_coord + 35)
    {
      stroke(255, 0, 0);
      line(i+x_coord, accel_x[i-1], (i+1)+x_coord, accel_x[i]);
    };
  }
};

// Acceleration Y
void Graph_accel_y_data(int x_coord, int y_coord, float data) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Data for drawing graph calculated to percents
  int max_data = 100;
  int draw_data = int (y_coord + 230 - ((190 / max_data) * data)) - 1;

  accel_y[Graph_width-1] = draw_data;

  // Shifting data in buffer
  for (int i = 0; i < Graph_width-1; i++)
  {
    accel_y[i] = accel_y[i+1];
  }

  // Data line (y)
  strokeWeight(1);
  for (int i = Graph_width-40; i > 20; i--)
  {
    if (accel_y[i-1] > y_coord + 35)
    {
      stroke(255, 0, 0);
      line(i+x_coord, accel_y[i-1], (i+1)+x_coord, accel_y[i]);
    };
  }
};

// Acceleration Z
void Graph_accel_z_data(int x_coord, int y_coord, float data) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Data for drawing graph calculated to percents
  int max_data = 100;
  int draw_data = int (y_coord + 230 - ((190 / max_data) * data)) - 1;

  accel_z[Graph_width-1] = draw_data;

  // Shifting data in buffer
  for (int i = 0; i < Graph_width-1; i++)
  {
    accel_z[i] = accel_z[i+1];
  }

  // Data line (y)
  strokeWeight(1);
  for (int i = Graph_width-40; i > 20; i--)
  {
    if (accel_z[i-1] > y_coord + 35)
    {
      stroke(255, 0, 0);
      line(i+x_coord, accel_z[i-1], (i+1)+x_coord, accel_z[i]);
    };
  }
};

// X-Y possition
void Graph_xy_poss_data(int x_coord, int y_coord, int data) // Coordinate x, Coordinate y, Data, Max data for calculation of percent, name of graph
{
  // Data for drawing graph calculated to percents
  int max_data = 200;
  int draw_data = 0;

  if (data >= 0) {
    draw_data = int (y_coord + 130 - ((115 / max_data) * data)) - 1;
  } else {
    draw_data = int (y_coord + 130 - ((115 / max_data) * data)) - 1;
  };

  xy_position[Graph_width-1] = draw_data;

  // Shifting data in buffer
  for (int i = 0; i < Graph_width-1; i++)
  {
    xy_position[i] = xy_position[i+1];
  }

  // Data line (y)
  strokeWeight(1);
  for (int i = Graph_width-40; i > 20; i--)
  {
    if (xy_position[i-1] > y_coord + 35)
    {
      stroke(255, 0, 0);
      line(i+x_coord, xy_position[i-1], (i+1)+x_coord, xy_position[i]);
    };
  }
};



void draw() {

  /************* Background *************/
  background(100, 100, 100);
  image(logo, 20, 20, logo.width/3.5, logo.height/3.5);

  textSize(28);
  fill(0, 0, 0);
  textFont(nasa);
  text("Ground control", 70, 80);


  /************* Areas *************/
  //rect(x, y, a, b, r)
  //rect(x, y, a, b, tl, tr, br, bl)

  // 1 - basic data
  stroke(0, 0, 0);
  fill(150, 150, 150);
  rect(20, 90, 220, 78, 5);


  /*** 1. box ***/
  // a - time current (Slovenija / Ljubljana)
  fill(0, 0, 0);
  int seconds = second();
  int minutes = minute();
  int hours = hour();
  text("Time:", 28, 115);

  if (hours < 10)
  {
    text("0", 89, 115);
    text(hours, 103, 115);
  } else
  {
    text(hours, 89, 115);
  };

  text(":", 118, 115);

  if (minutes < 10)
  {
    text("0", 124, 115);
    text(minutes, 138, 115);
  } else
  {
    text(minutes, 125, 115);
  };

  text(":", 153, 115);

  if (seconds < 10) {
    text("0", 160, 115);
    text(seconds, 175, 115);
  } else
  {
    text(seconds, 160, 115);
  };


  // b - time UTC (current - 2)
  text("UTC:", 34, 135);

  int hoursUTC = hours - 2;
  if (hoursUTC < 10)
  {
    text("0", 89, 135);
    text(hoursUTC, 103, 135);
  } else
  {
    text(hoursUTC, 89, 135);
  };

  text(":", 118, 135);

  if (minutes < 10) {
    text("0", 124, 135);
    text(minutes, 138, 135);
  } else
  {
    text(minutes, 125, 135);
  };

  text(":", 153, 135);

  if (seconds < 10)
  {
    text("0", 160, 135);
    text(seconds, 175, 135);
  } else
  {
    text(seconds, 160, 135);
  };


  // c - date
  int days = day();
  int months = month();
  int years = year();
  text("Date:", 28, 155);
  if (days < 10)
  {
    text("0", 89, 155);
    text(days, 103, 155);
  } else
  {
    text(days, 88, 155);
  };

  text(":", 118, 155);

  if (months < 10)
  {
    text("0", 124, 155);
    text(months, 138, 155);
  } else
  {
    text(months, 125, 155);
  };

  text(":", 153, 155);
  text(years, 160, 155);

  // 2 - Status
  textSize(20);
  fill(255, 255, 255);
  text("Status:", 25, 195);

  stroke(0, 0, 0);
  fill(150, 150, 150);
  rect(20, 200, 220, 110, 5);

  textSize(13);
  fill(0, 0, 0);
  text("Main bat.:", 28, 220);
  main_bat = 31.73;  // !! delete !!
  String main_bat_s = nf(main_bat, 2, 1);
  if (main_bat < 25.6)   // enter critical battery value (3.0v * No. cell) (3.2V) ; 3.2 * 8 = 25,6
  {
    fill(255, 0, 0);
  } else
  {
    fill(0, 0, 0);
  };

  text(main_bat_s, 140, 220);
  fill(0, 0, 0);
  text("V", 170, 220);

  text("EDF bat.:", 28, 233);
  EDF_bat = 31.73;  // !! delete !!
  String EDF_bat_s = nf(EDF_bat, 2, 1);
  if (EDF_bat < 25.6)   // enter critical bat value (3.0v * No. cell) (3.2V) ; 3.2*8 = 25,6
  {
    fill(255, 0, 0);
  } else
  {
    fill(0, 0, 0);
  };

  text(EDF_bat_s, 140, 233);
  fill(0, 0, 0);
  text("V", 170, 233);

  text("Status:", 28, 246);
  text(amon_status, 150, 246);

  text("Connection:", 28, 259);
  if (connection_ok == 0)
  {
    text("NC", 145, 259);
  } else
  {
    text("OK", 145, 259);
  };

  fill(0, 0, 0);
  text("Temp.:", 28, 272);
  temp = 35.73;  // !! delete !!
  String temp_s = nf(temp, 2, 1);
  text(temp_s, 140, 272);
  text("C", 170, 272);

  text("Press.:", 28, 285);
  press = 1013.25;  // !! delete !!
  String press_s = nf(press, 2, 1);
  text(press_s, 140, 285);
  text("hPa", 180, 285);

  text("Hum.:", 28, 298);
  hum = 50.73;  // !! delete !!
  String hum_s = nf(hum, 2, 1);
  text(hum_s, 140, 298);
  text("%", 170, 298);

  // 3 - indicator  !! dodaj !!
  stroke(0, 0, 0);
  amon_status = 3; // !! delete !!
  // Change color
  if (amon_status == 0) // starting
  {
    fill(150, 150, 150);
  };

  if (amon_status == 1) // Emergency stop
  {
    fill(255, 0, 0);
  }
  if (amon_status == 2) // Ready
  {
    fill(0, 255, 0);
  }
  if (amon_status == 3) // Launch
  {
    fill(0, 0, 255);
  }

  rect(55, 325, 150, 35, 5);

  // Change text
  fill(0, 0, 0);
  textSize(18);
  if (amon_status == 0) // starting
  {
    text("STARTING", 82, 349);
  };
  if (amon_status == 1) // Emergency stop
  {
    text("STOP", 105, 349);
  }
  if (amon_status == 2) // Ready
  {
    text("READY", 100, 349);
  }
  if (amon_status == 3) // Launch
  {
    text("LAUNCH", 90, 349);
  }

  // 4 - terminal
  textSize(18);
  fill(255, 255, 255);
  text("Terminal:", 25, 385);

  stroke(0, 0, 0);
  fill(10, 10, 10);
  rect(20, 390, 220, 270, 5);

  // a - terminal text
  textFont(txtDef);
  textSize(15);
  fill(255, 255, 255);
  text("Test:", 28, 410);


  /*** 2. column ***/
  // 5 - raw telemetry
  textFont(nasa);
  textSize(18);
  fill(255, 255, 255);
  text("Raw telemetry:", 275, 35);

  stroke(0, 0, 0);
  fill(150, 150, 150);
  rect(260, 40, 220, 380, 5);

  // 6 - TVC Data
  stroke(0, 0, 0);
  fill(150, 150, 150);
  rect(260, 430, 220, 160, 5);

  textFont(nasa);
  textSize(23);
  fill(255, 255, 255);
  text("TVC", 350, 455);
  line(270, 465, 470, 465);

  textSize(18);
  text("X+:", 270, 490);
  text("X-:", 380, 490);
  text("Y+:", 270, 530);
  text("Y-:", 380, 530);
  
  text("°", 360, 490);
  text("°", 460, 490);
  text("°", 360, 530);
  text("°", 460, 530);
  
  line(270, 545, 470, 545);
  
  text("Status:", 270, 573);
  if (amon_status == 3){
    text("Active", 350, 573);
  }else{
    text("OFF", 350, 573);
  };
  
  /************* MENU Button *************/
  stroke(0, 0, 0);
  update(); // mouseX, mouseY
  
  if (OverBtn(260, 610, 220, 50)) {
    fill(0, 150, 0);
  } else {
    fill(50, 50, 50);
  }
  rect(260, 610, 220, 50, 5);
  
  textFont(nasa);
  textSize(25);
  fill(255, 255, 255);
  text("MENU", 330, 643);

  /************* Graphs *************/

  //delay(5); // Delay to slow down time of graph drawing

  testData++; // brisi
  if (testData == 100) testData = 0; // brisi

  if (updn == 0) {
    testDataNeg++; // brisi
    if (testDataNeg == 100) updn = 1;
  }

  if (updn == 1) {
    testDataNeg--; // brisi
    if (testDataNeg == -100) updn = 0;
  }


  //if (millis()%2 == 0){

  // EDF throtle [% - sec]
  Graph_percent_show(40, 680, "EDF Throtle");
  Graph_percent_data(40, 680, testData);

  // Altitude [m - sec]
  Graph_meter_show(510, 40, "Altitude (m)");
  Graph_altitude_data(510, 40, testData);

  // X-Y position [m - sec]
  Graph_meter_center_show(980, 40, "X-Y Position (m)");
  Graph_xy_poss_data(980, 40, testDataNeg);

  // X-Y velocity [m/s - sec]
  Graph_meter_sec_show(1450, 40, "X-Y Velocity (m/s)");
  Graph_velocity_data(1450, 40, testData);

  // Angle X [degres - sec]
  Graph_degres_show(510, 360, "Angle X (deg)");

  // Angle Y [degres - sec]
  Graph_degres_show(980, 360, "Angle Y (deg)");

  // Angle Z [degres - sec]
  Graph_degres_show(1450, 360, "Angle Z (deg)");

  // Acceleration X [m/s^2 - sec]
  Graph_meter_meter_sec_show(510, 680, "Acceleration X (m/s^2)");
  Graph_accel_x_data(510, 680, testData);

  // Acceleration Y [m/s^2 - sec]
  Graph_meter_meter_sec_show(980, 680, "Acceleration Y (m/s^2)");
  Graph_accel_y_data(980, 680, testData);

  // Acceleration Z [m/s^2 - sec]
  Graph_meter_meter_sec_show(1450, 680, "Acceleration Z (m/s^2)");
  Graph_accel_z_data(1450, 680, testData);

  //}
  
  //************* SERIAL *************//
  /*
   while (myPort.available() > 0) {
     int inByte = myPort.read();
     println(inByte);
  }
  */
}

// Menu Btn update
void update() {
  if (OverBtn(260, 610, 220, 50)) {
    btnOver = true;
  } else {
    btnOver = false;
  }
}

// Over btn detect
boolean OverBtn(int x, int y, int width, int height)  {
  if (mouseX >= x && mouseX <= x+width && mouseY >= y && mouseY <= y+height) {
    return true;
  } else {
    return false;
  }
}


void mousePressed() {
  if (OverBtn(260, 610, 220, 50)) {
    WindowA = new MenuWindow(3, 500, 353, 400, 300);
  }
}


class MenuWindow extends PApplet {
  int id, vx, vy, vw, vh;

  MenuWindow(int id, int vx, int vy, int vw, int vh) {
    super();
    this.id = id;
    this.vx = vx;
    this.vy = vy;
    this.vw = vw;
    this.vh = vh;

    PApplet.runSketch(new String[] { this.getClass().getName() }, this);
  }

  void settings() {
    size(600, 400);
    smooth(0);
  }

  void setup() {
    surface.setLocation(vx, vy);
    
    // check serial
    ports = Serial.list();
    if (ports > 0){
      println("Available COM ports found!");
    }else{
        println("No available COM ports found!");
    }
  }

  void draw() {
    background(100, 100, 100);
    
    textFont(nasa);
    textSize(15);
    fill(255, 255, 255);
    text("COM:", 20, 20);
    
    

    
    
    WindowA.stop(); // ??
  }
}
