package flytrap;
import Engine.*;

import java.util.*;

import lejos.hardware.Button;

public class Flytrap {
	
	public static RConsole rcon = new RConsole();
	
	public static void sleep(long amount) {
		try {
			Thread.sleep(amount);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	public static void sendPoint(int type, int x, int y) {
		rcon.out.println(type + " " + x + " " + y);
	}
	
	// get information from remote 
	public static void getPose() {
		rx = (double)Integer.parseInt(rcon.waitLine());
		ry = (double)Integer.parseInt(rcon.waitLine());
		heading = (double)Integer.parseInt(rcon.waitLine());
	}
	public static void getTargets() {
		while (true) {
			Target cur_target = new Target();
			cur_target.type = Integer.parse(rcon.waitLine());
			cur_target.x = Integer.parseInt(rcon.waitLine());
			cur_target.y = Integer.parseInt(rcon.waitLine());
			cur_target.theta = Integer.parseInt(rcon.waitLine());
			if (cur_target.x == 0 && cur_target.y == 0) break;
			targets.add(cur_target);
		}
	}


	public static void main(String[] args) {
		EmergencyStop.start();
		rcon.start();
		// wait to be connected
		while (!rcon.connected) sleep(100);

		getPose();
		getTargets();

		// testing ECHO to rconsole
		for (int i = 300; i < 400; i++) {
			sendPoint(4, i, i);
			sleep(100);
		}


		go();
	}

}
