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
	public static void sendPose() {
		rcon.out.println(active_behaviour + " " + (int)rx + " " + (int)ry + " " + (int)heading);
	}
	
	// get information from remote 
	public static void getPose() {
		rx = (double)Integer.parseInt(rcon.waitLine());
		ry = (double)Integer.parseInt(rcon.waitLine());
		heading = (double)Integer.parseInt(rcon.waitLine());
		
		sendPoint(9, (int)rx, (int)ry);	// acknowledge
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
		target = targets.get(targets.size() - 1);
	}

	
	// usable modules to be put into main
	public static void gameplay() {
		getPose();
		getTargets();
		for (Target targ: targets) sendPoint(0, targ.x, targ.y);

		motor_l.resetTachoCount();
		motor_r.resetTachoCount();
		
		
		while (on) {
			long start = System.currentTimeMillis();
			go();
			sendPose();
			long delta = System.currentTimeMillis() - start;
			sleep(Math.max(0, CYCLE_PERIOD - delta));
		}
	}
	// debugging modules 
	public static void debug_PID() {
		final int set_speed = 100 / CYCLES_PER_SEC;
		target_l = target_r = set_speed;
		rcon.out.println("! " + set_speed*CYCLES_PER_SEC + " " + Engine.KP + "_" + Engine.KI + "_" + Engine.KD);
		
		motor_l.resetTachoCount();
		motor_r.resetTachoCount();
		
		for (int b = 0; b < NUM_BEHAVIOUR; ++b) {
			disable_behaviour(b);
		}
		while (on) {
			long start = System.currentTimeMillis();
			go();
			rcon.out.println("! " + (instant_speed_l * 1000 / CYCLE_PERIOD) + " " + (instant_speed_r * 1000 / CYCLE_PERIOD));
			long delta = System.currentTimeMillis() - start;
			sleep(Math.max(0, CYCLE_PERIOD - delta));
		}	
		
	}
	public static void main(String[] args) {
		EmergencyStop.start();
		rcon.start();
		on = true;
		// wait to be connected
		while (!rcon.connected) sleep(100);

		gameplay();
//		debug_PID();
	}

}
