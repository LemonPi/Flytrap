package flytrap;
import static flytrap.Engine.*;

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
		sendPoint(0, (int)rx, (int)ry);
	}
	public static void getTargets() {
		while (true) {
			Target cur_target = new Target();
			cur_target.type = Integer.parseInt(rcon.waitLine());
			cur_target.x = Integer.parseInt(rcon.waitLine());
			cur_target.y = Integer.parseInt(rcon.waitLine());
			cur_target.angle = Integer.parseInt(rcon.waitLine());
			if (cur_target.x == 0 && cur_target.y == 0) break;
			targets.add(cur_target);
		}
		behaviours[NAV_LAYER].active = true;
	}


	public static void main(String[] args) {
		EmergencyStop.start();
		rcon.start();
		// wait to be connected
		while (!rcon.connected) sleep(100);

		getPose();
		getTargets();
		for (Target targ: targets) sendPoint(0, targ.x, targ.y);

		motor_l.resetTachoCount();
		motor_r.resetTachoCount();
		
		while (true) {
			long start = System.currentTimeMillis();
			go();
			sendPoint(active_behaviour, (int)rx, (int)ry);
			long delta = System.currentTimeMillis() - start;
			sleep(Math.max(0, 50 - delta));
		}
	}

}
