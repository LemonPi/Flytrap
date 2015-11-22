import processing.net.*;
import java.util.ArrayDeque;
double PIXEL_PER_MM = 2;
int offset_x = -200;
int offset_y = 800;
double BOUNDARY_SIZE_SCALE = 2*PIXEL_PER_MM;
int GRID_WIDTH = 200;

int x = 0;
int y = 0;
int target_x = 0;
int target_y = 0;
int target_theta = 0;  // degrees clamped between -180 and 180

ArrayDeque<String> line_queue = new ArrayDeque<String>();
// 3-tuple (x,y,theta)
ArrayDeque<int[]> targets = new ArrayDeque<int[]>();

Server brain;

void setup() {
  noSmooth();
  
  brain = new Server(this, 3334);
  if (brain.active()) println("Server up and running");
  
  size(2000, 1600);
  draw_grid();
    
  strokeWeight(5);
}

void give_pose() {
    brain.write("0\n");
    brain.write("0\n");
    brain.write("0\n");
    println("gave pose");
}

void draw() {
  Client flytrap = brain.available();
  if (flytrap != null) {
    String msg = flytrap.readString();
    if (msg != null) {
      print(msg);
      String trimmed_msg = msg.trim();
      if (trimmed_msg.length() >= 1)
        process_msg(trimmed_msg);
    }
  }
}

void process_msg(String msg) {
  // put message onto a map with appropriate marker
  switch (msg.charAt(msg.length()-1)) {
    case 'X': {
      // target
    }
    case 'B': {
      // boundary
    }
    default: {
      if (msg.charAt(0) == '-') return;  // debug message
      String[] params = splitTokens(msg);
      if (params.length < 3) {
        println("bad format");
        return;
      }
      x = int(params[1]);
      y = int(params[2]);
      // active layer is params[0]
      switch (msg.charAt(0)) {
        case '9':  stroke(255,105,180); strokeWeight(15); break;  // for initial pose
        case '4':  stroke(0,0,250); break;
        case '3':  stroke(0,250,0); break;
        case '2':  stroke(0); break;
        case '1':  stroke(250,250,0); break;
        case '0':  stroke(250); break;
      }
      point((int)(PIXEL_PER_MM*(y+offset_y)), (int)(height - PIXEL_PER_MM*(x-offset_x)));
      //println((int)(PIXEL_PER_MM*(y+offset_y)), (int)(height - PIXEL_PER_MM*(x-offset_x)));

      strokeWeight(7);  // default stroke weight
    }
  }
}

boolean creating_target = false;
void mouseReleased() {
  stroke(255);
  strokeWeight(10);
  int tx = (int)((height - (mouseY-offset_x))/PIXEL_PER_MM);
  int ty = (int)((mouseX-offset_y)/PIXEL_PER_MM);
  // in the process of creating a target, second click sets the orientation
  if (creating_target) {
    double tt = Math.atan2(ty-target_y, tx-target_x);
    // convert to degrees
    target_theta = (int)(tt * 180 / Math.PI);
    // clamp to [-180,180]
    if (target_theta > 180) target_theta -= 360;
    else if (target_theta < -180) target_theta += 360;
    
    targets.push(new int[]{target_x, target_y, target_theta,4});
    println(target_x,target_y,target_theta);    
    creating_target = false;
  }
  else {
    target_x = tx;
    target_y = ty;
    point(mouseX, mouseY);
    creating_target = true;
  }

}

void keyPressed() {
  if (key == 's') {
    String savename = new String();
    savename += target_x;
    savename += '_';
    savename += target_y;
    saveFrame(savename);
  }
  else if (key == '1') {
    println("sending prebuilt targets");
    brain.write("4\n170\n-100\n0\n");
    //brain.write("4\n500\n0\n0\n");
    brain.write("4\n225\n-8\n0\n");
    brain.write("0\n0\n0\n0\n");
  }
  else if (key == '0') {
    println("sending " + targets.size() + " targets");
    while (!targets.isEmpty()) {
      int[] whatever = targets.pop();
      brain.write(Integer.toString(whatever[3]) + "\n");
      brain.write(Integer.toString(whatever[0]) + "\n");
      brain.write(Integer.toString(whatever[1]) + "\n");
      brain.write(Integer.toString(whatever[2]) + "\n");
    }
    // terminate input sequence?
    brain.write("0\n0\n0\n0\n");
  }
  else if (key == 'p') {
    give_pose();
  }
}


  
void draw_grid() {
  stroke(150);
  for (int grid_x = 0; grid_x < height; grid_x += GRID_WIDTH*PIXEL_PER_MM)
    line(0,grid_x, width,grid_x);
    
  for (int grid_y = 0; grid_y < width; grid_y += GRID_WIDTH*PIXEL_PER_MM)
    line(grid_y, 0, grid_y, height);
}