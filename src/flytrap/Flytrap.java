package flytrap;
import static flytrap.Engine.*;

import lejos.hardware.Button;
import lejos.hardware.Key;
import lejos.hardware.motor.NXTRegulatedMotor;

public class Flytrap {

	public static RConsole rcon = new RConsole();
	
	public static void sleep(long amount) {
		try {
			Thread.sleep(amount);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	public static void wait_for_keypress(Key key) {
		while (!key.isDown()) {
			sleep(100);
		}
	}
	public static void sendPoint(int type, int x, int y) {
		rcon.out.println(type + " " + x + " " + y);
	}
	public static void sendPose() {
		Behaviour active_b = behaviours[active_behaviour];
		rcon.out.println(active_behaviour + " " + (int)rx + " " + (int)ry + " " + (int)heading + " | " + active_b.speed + " " + active_b.turn);
	}
	public static void sendBoundary(int x, int y, int r, int active) {
		rcon.out.println(x + " " + y + " " + r + " " + active + " B");
	}
	public static void sendTarget(int type, int x, int y, int angle) {
		rcon.out.println(type + " " + x + " " + y + " " + angle + " X");
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
			int type = Integer.parseInt(rcon.waitLine());
			int x = Integer.parseInt(rcon.waitLine());
			int y = Integer.parseInt(rcon.waitLine());
			int angle = Integer.parseInt(rcon.waitLine());
			if (x == 0 && y == 0) break;
			add_target(x, y, angle, type);
		}                                                               
		behaviours[NAV_LAYER].active = true;
		if (!targets.isEmpty())
			target = targets.get(targets.size() - 1);
	}
	
	
	// runnable classes for go to use
	static class ReportPose implements Runnable {
		public void run() {sendPose();}
	}
	static class ReportPID implements Runnable {
		public void run() {rcon.out.println("! " + (instant_speed_l * 1000 / CYCLE_PERIOD) + " " + (instant_speed_r * 1000 / CYCLE_PERIOD));}
	}
	
	
	// usable modules to be put into main
	public static void gameplay() {
		getPose();
		while (targets.isEmpty())
			getTargets();
		for (Target targ: targets) sendTarget(targ.type, targ.x, targ.y, targ.angle);

		motor_l.resetTachoCount();
		motor_r.resetTachoCount();
		
//		disable_behaviour(NAV_LAYER);
//		disable_behaviour(TURN_LAYER);
//		disable_behaviour(BOUNDARY_LAYER);
//		disable_behaviour(GET_LAYER);
//		drive_mode = MANUAL;
//		for (int i = 0; i < 300; i += 60)
			AvoidBoundary.add_boundary((int)rx + 200, (int)ry + 40, 40);
		Runnable reporter = new ReportPose();
		
		wait_for_keypress(Button.DOWN);
		
		while (on) {
			go(reporter);
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
		
		Runnable reporter = new ReportPID();

		while (on) {			
			go(reporter);
		}	
		
	}
	public static void debug_turn_in_place() {
		motor_l.resetTachoCount();
		motor_r.resetTachoCount();
		
		motor_l.setSpeed(200);
		motor_r.setSpeed(200);
		
		disable_behaviour(BOUNDARY_LAYER);
		
		getPose();
		
		add_target((int)rx, (int)rx, 0, TURN_LAYER);
		add_target((int)rx, (int)ry, 180, TURN_LAYER);
		
		Runnable reporter = new ReportPose();

		while (on) {
			go(reporter);
		}
	}
	public static void debug_boundary_avoid() {
		while (on) {
			rcon.out.println(AvoidBoundary.sense_boundary());
			sleep(50);
		}
	}

	
	
	public static void main(String[] args) {
		EmergencyStop.start();
		new Thread(new Runnable() {
			public void run() {
				AvoidBoundary.init();
			}
		}).start();
		rcon.start();
		on = true;
		// wait to be connected
		while (!rcon.connected) sleep(100);

		gameplay();
//		debug_PID();
//		debug_turn_in_place();
//		debug_boundary_avoid();
	}
	
	
	
	
	// calibration without direct server report
	public static void debug_calibration() {
		EmergencyStop.start();
		while (!Button.ENTER.isDown()) {
			sleep(100);
		}
		motor_l.resetTachoCount();
		motor_r.resetTachoCount();
		
		motor_l.synchronizeWith(new NXTRegulatedMotor[]{motor_r});
		motor_l.startSynchronization();
		
//		int rot_cycles = 5;
//		motor_l.rotate(360*rot_cycles);
//		motor_r.rotate(360*rot_cycles);

		int rot = 712 * 2;
		motor_l.rotate(rot);
		motor_r.rotate(-rot);
		
		motor_l.endSynchronization();
		
		while (true) {
			sleep(100);
		}
	}
	
	public static void mainn(String[] args) {
		EmergencyStop.start();
		on = true;
		GetBall.clawMotor.resetTachoCount();
		GetBall.liftMotor.resetTachoCount();
		
		int pos = 0;
//		debug_calibration();
		while (on) {
//			System.out.println("c" + GetBall.clawMotor.getTachoCount() + ":" +
//					GetBall.liftMotor.getTachoCount());
			
			if (Button.DOWN.isDown()) {pos = (pos - 1) % 4; GetBall.lift_claw(pos);}
			else if (Button.UP.isDown()) {pos = (pos + 1) % 4; GetBall.lift_claw(pos);}
			else if (Button.LEFT.isDown()) GetBall.open_claw();
			else if (Button.RIGHT.isDown()) GetBall.close_claw();
			sleep(100);
		}
	}

}
