import processing.net.*;

final int CYCLES = 300;  // watch for this many PID cycles
final int PIXEL_PER_CYCLE = 2;  // on a graph, the increment of a cycle in pixels
final int MINSPEED = 0;
final int MAXSPEED = 600;

Server brain;
// horizontal position, used as index
int pos_l = 0;
int pos_r = 0;
int prev_l = 0;
int prev_r = 0;
int[] ticks_l = new int[CYCLES];
int[] ticks_r = new int[CYCLES];

//Variables to draw a continuous line.

int target;  // ms between ticks
String setpoint;
String params;  // kp ki kd

boolean init = false;
boolean drawn_l = false;
boolean drawn_r = false;
Client myPort;

void setup () {
  // set the window size:
  size(600, 600);      // 600 pixels high; each cycle is PIXEL_PER_CYCLE wide
  background(0);      // set inital background: 



  strokeWeight(4);        //stroke width for response curve
  
  
  // List all the available serial ports
  brain = new Server(this, 3334);
  if (brain.active()) println("Server up and running");

}
void serverEvent(Server server, Client client) {
  println("Client connected");
  myPort = client;
}
boolean doinit(String inString) {
  if (init) return false;
  String[] parts = splitTokens(trim(inString));
  if (parts[0].charAt(0) != '!') return false;
  setpoint = parts[1];
  println(setpoint);
  target = int(setpoint);
  params = parts[2];
  println(params);
  println("client inited");
  init = true;
  
  // target line
  target = height - (int)map(target, MINSPEED, MAXSPEED, 0, height);
  strokeWeight(10); 
  stroke(100);
  line(0, target, width, target);
  
  
  strokeWeight(4);
  // A serialEvent() is generated when a newline character is received :
  //myPort.bufferUntil('\n');

  return true;
}
void draw () {
  myPort = brain.available();
  if (myPort == null) return;
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');
  if (inString == null) return;
  if (doinit(inString)) return;
  if (!init) return;
  
  // finished cycle testing
  if (pos_l >= CYCLES-1 && pos_r >= CYCLES-1) {
    save_map();
    myPort.stop();
    brain.stop();
    exit();
    double dev_l = deviation(ticks_l, target);
    double dev_r = deviation(ticks_r, target);
    double avg_l = average(ticks_l);
    double avg_r = average(ticks_r);
    print("Average left: ");
    print(avg_l);
    print("\tDeviance left: ");
    println(dev_l);
    
    print("Average right: ");
    print(avg_r);
    print("\tDeviance right: ");
    println(dev_r);
  }
  
  if (inString != null) {
    print(inString);
    String[] params = splitTokens(inString);
    if (params[0].charAt(0) != '!') return;

    if (pos_l < CYCLES) {
      int ticks = Integer.parseInt(params[1]);
      ticks_l[pos_l] = ticks;
      ++pos_l;
      stroke(127, 34, 255);     //stroke color

      // use height - since pixel count is 0 at the top left corner...
      int prev_height = height - int(map(prev_l, MINSPEED, MAXSPEED, 0, height));
      int graph_height = height - int(map(ticks, MINSPEED, MAXSPEED, 0, height));
      line((pos_l-1)*PIXEL_PER_CYCLE, prev_height, pos_l*PIXEL_PER_CYCLE, graph_height);
      prev_l = ticks;
    } 
    if (pos_r < CYCLES) {
      int ticks = Integer.parseInt(params[2]);
      ticks_r[pos_r] = ticks;
      ++pos_r;
      stroke(127, 34, 0);     //stroke color

      // plot all the right ticks
      // use height - since pixel count is 0 at the top left corner...
      int prev_height = height - int(map(prev_r, MINSPEED, MAXSPEED, 0, height));
      int graph_height = height - int(map(ticks, MINSPEED, MAXSPEED, 0, height));
      line((pos_r-1)*PIXEL_PER_CYCLE, prev_height, pos_r*PIXEL_PER_CYCLE, graph_height);
      prev_r = ticks;
    }
  }
}

// mean deviation (signed) from the target for the latter 5/6 of the cycle
double deviation(int[] values, int target) {
  int tot = 0;
  for (int i = CYCLES/6; i < CYCLES; ++i) {
    tot += values[i] - target;
  }
  return ((double) tot) / (5*CYCLES/6);
}

double average(int[] values) {
  int tot = 0;
  for (int i = CYCLES/6; i < CYCLES; ++i) {
    tot += values[i];
  }
  return ((double) tot) / (5*CYCLES/6);
}

// save image upon key press of s
void keyPressed() {
  if (key == 's') {
    save_map();
  }
}

void save_map() {
    String filename = setpoint;
    filename += '_';
    params = params.replace('.', '_');
    filename += params;
    saveFrame(filename);
}