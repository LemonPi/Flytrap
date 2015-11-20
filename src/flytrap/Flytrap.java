package flytrap;

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
	
	public static void getPose() {
		rx = (double)Integer.parseInt(rcon.waitLine());
		ry = (double)Integer.parseInt(rcon.waitLine());
		heading = (double)Integer.parseInt(rcon.waitLine());

	}

	public static void getTargets() {
		while (true) {
			int x = Integer.parseInt(rcon.waitLine());
			int y = Integer.parseInt(rcon.waitLine());
			int angle = Integer.parseInt(rcon.waitLine());
			if (x == 0 && y == 0) break;
			targets.add(new int[]{x, y, angle});
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
