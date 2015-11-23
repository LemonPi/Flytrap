package flytrap;
import java.util.*;
import lejos.hardware.motor.*;

import static flytrap.Engine.CYCLE_PERIOD;
import static flytrap.Engine.target;
import static flytrap.Engine.targets;
import static java.lang.Math.*;

public class Engine {
	// constants 
	public static final int CYCLE_PERIOD = 50;	// ms
	public static final int CYCLES_PER_SEC = 1000 / CYCLE_PERIOD;
	
	public static final double KP = 1;
	public static final double KI = 0;
	public static final double KD = 0;
//	public static final double KP = 1.194;
//	public static final double KI = 1.2;
//	public static final double KD = 0.005;
	public static final int FWD = 1;
	public static final int BWD = -1;
	public static final double MM_PER_DEGREE = 134.0/360;
	public static final double BASE_WIDTH = 124.406;
//	public static final double MM_PER_DEGREE = 3470/20.0/360.0;
//	public static final double BASE_WIDTH = (90+145)/2;

	
	public static final int MIN_SPEED = 120 / CYCLES_PER_SEC;
	public static final int TOP_SPEED = 460 / CYCLES_PER_SEC;	// degrees per second

	public static final int ANY_HEADING = 9000;
	public static final double TARGET_CLOSE_ENOUGH = 40;	// in mm
	public static final double TARGET_IMMEDIATE = 5;
	public static final double TARGET_CLOSE = 500;	// in mm
	public static final double HEADING_TOLERANCE = 3;	// in angle
	public static final double HEADING_THRESHOLD_TURN_IN_PLACE = 40;	// in angle

	public static final int NUM_BEHAVIOUR = 6;
	public static final int BOUNDARY_LAYER = 0;
	public static final int GET_LAYER = 1;	// gets activated when near ball dispensers without ball
	public static final int PUT_LAYER = 2;	// gets activated when near ball repostitory with ball
	public static final int TURN_LAYER = 3;	// turning in place
	public static final int NAV_LAYER = 4;	// otherwise on
	public static final int WAIT_LAYER = 5;
	
	public static final int TARGET_GET = GET_LAYER; // used to mark a target as a ball pickup or dropoff target
	
	// internal state
	static boolean on = false;
	static NXTRegulatedMotor motor_l = Motor.A;
	static NXTRegulatedMotor motor_r = Motor.B;

	// target speeds (angles per cycle)
	static int target_l = 0;
	static int target_r = 0;
	// instantaneous speeds
	static int instant_speed_l = 0;
	static int instant_speed_r = 0;

	// PID related stuff
	static int prev_l, prev_r;
	static double integral_l, integral_r;
	static int out_l, out_r;

	static Behaviour[] behaviours = new Behaviour[NUM_BEHAVIOUR];
	static {
		for (int i = 0; i < behaviours.length; i++) {
			behaviours[i] = new Behaviour();
			behaviours[i].active = false;
		}
	}
	static int active_behaviour = WAIT_LAYER;	// initially no available layer
	static int allowed_behaviours = ~0;	// bit field with 0 being allowed and 1 being disabled

	static double rx,ry,heading;	// heading is in degrees [-180,180]

	static List<Target> targets = new ArrayList<Target>();
	static Target target = null;

	static int process_cycles;
	static int wait_cycles = 0;


	public static <T extends Comparable<T>> T clamp(T val, T min, T max) {
	    if (val.compareTo(min) < 0) return min;
	    else if (val.compareTo(max) > 0) return max;
	    else return val;
	}
	public static double clamp(double val, double min, double max) {
		return val < min? min: val > max? max: val;
	}
	public static int clamp(int val, int min, int max) {
		return val < min? min: val > max? max: val;
	}
	public static double wrap_angle(double angle) {
		return angle < -180? angle + 360: angle > 180? angle - 360: angle;
	}
	public static int wrap_angle(int angle) {
		return angle < -180? angle + 360: angle > 180? angle - 360: angle;
	}
	public static double wrap_rad(double rad) {
		return rad < -Math.PI? rad + 2*Math.PI: rad > Math.PI? rad - 2*Math.PI: rad;
	}
	public static double rad(double deg) {
		return Math.toRadians(deg);
	}
	public static double deg(double rad) {
		return Math.toDegrees(rad);
	}
	public static double sq(double a) {
		return a*a;
	}
	
	public static boolean allowed_behaviour(int b) {
		return ((1 << b) & allowed_behaviours) != 0;
	}
	public static void allow_behaviour(int b) {
		allowed_behaviours |= (1 << b);
	}
	public static void disable_behaviour(int b) {
		allowed_behaviours &= ~(1 << b);
	}
	
	public static int add_target(int x, int y, int angle, int type) {
		target = new Target();
		target.x = x;
		target.y = y;
		angle = wrap_angle(angle);
		target.angle = angle;
		target.type = type;
		targets.add(target);
		Flytrap.sendTarget(type, x, y, angle);
		
		behaviours[NAV_LAYER].active = true;
		
		return targets.size();
	}

	public static void pid_control(int sl, int sr) {
//		int err_l = target_l - sl;
//		int err_r = target_r - sr;
//
//		// derivative on measurement to fix derivative kick
//		int input_change_l = (sl - prev_l);
//		int input_change_r = (sr - prev_r);
//
//		// update state
//		prev_l = sl;
//		prev_r = sr;
//
//		// integral
//		integral_l += KI * err_l;
//		integral_r += KI * err_r;
//		clamp(integral_l, 0, 255);
//		clamp(integral_r, 0, 255);
//
//		out_l = (int) (KP*err_l + integral_l - KD*input_change_l);
//		out_r = (int) (KP*err_r + integral_r - KD*input_change_r);
		out_l = target_l;
		out_r = target_r;
	}
	
	public static void hard_break(int behaviour, int cycles) {
		wait_cycles = cycles;
		motor_l.stop(true);
		motor_r.stop(true);
		Flytrap.rcon.out.println("h " + behaviour + "| " + cycles);
	}

	public static void odometry() {
		// get the displacement from motors (can read angular displacement directly)
		instant_speed_l = motor_l.getTachoCount();
		instant_speed_r = motor_r.getTachoCount();
		
		double displacement_l = instant_speed_l * MM_PER_DEGREE;
		double displacement_r = instant_speed_r * MM_PER_DEGREE;
		motor_l.resetTachoCount();
		motor_r.resetTachoCount();

		double displacement = (displacement_l + displacement_r) * 0.5;

		heading += Math.toDegrees(Math.atan2(displacement_l - displacement_r, BASE_WIDTH));
		heading = wrap_angle(heading);
		rx += displacement * cos(rad(heading));
		ry += displacement * sin(rad(heading));
	}

	public static void motor_control(int b) {
		Behaviour control_behaviour = behaviours[b];

		target_l = control_behaviour.speed + control_behaviour.turn;
		target_r = control_behaviour.speed - control_behaviour.turn;

	}

	public static void arbitrate() {
		// pick highest priority active behaviour and follow it's desires
		for (int b = 0; b < NUM_BEHAVIOUR; ++b) {
			if (behaviours[b].active) {
				if (b == WAIT_LAYER) {
					active_behaviour = b;
					hard_break(WAIT_LAYER, -1);
				}
				motor_control(b);
				active_behaviour = b;
				return;
			}
		}
	}

	public static boolean go(Runnable reporter) {
		long start = System.currentTimeMillis();

		odometry();

		if (wait_cycles != 0) {
			--wait_cycles;
		}
		else {
			process_cycles = 1;
			
			while (process_cycles > 0) {
				// process behaviours
				if (target != null) Navigate.locate_target();
				if (allowed_behaviour(NAV_LAYER)) Navigate.navigate();
				if (allowed_behaviour(TURN_LAYER)) Turn.turn_in_place();
				if (allowed_behaviour(BOUNDARY_LAYER)) AvoidBoundary.avoid_boundary();
				if (allowed_behaviour(GET_LAYER)) GetBall.get_ball();
				
				arbitrate();
				--process_cycles;
			}
			
			
			pid_control(instant_speed_l, instant_speed_r);
			
	//		Flytrap.rcon.out.println("- " + out_l + " " + out_r);
			
			motor_l.setSpeed(CYCLES_PER_SEC * abs(out_l));
			motor_r.setSpeed(CYCLES_PER_SEC * abs(out_r));
			
			if (out_l > 0) motor_l.forward();
			else if (out_l < 0) motor_l.backward();
			else motor_l.stop();
			
			if (out_r > 0) motor_r.forward();
			else if (out_r < 0) motor_r.backward();
			else motor_r.stop();		
		}
		
		reporter.run();
		
		long delta = System.currentTimeMillis() - start;
		Flytrap.sleep(Math.max(0, CYCLE_PERIOD - delta));
		return true;
	}

	public static void waypoint(int behaviour) {
		Flytrap.rcon.out.println("w " + behaviour);

		if (active_behaviour == TURN_LAYER) {
			hard_break(TURN_LAYER, 5);
		}
		
		if (targets.size() > 1) {
			// more targets to reach, get to those
			targets.remove(targets.size()-1);
			target = targets.get(targets.size() - 1);
			behaviours[NAV_LAYER].active = true;
		}
		// done, just exit
		else {
			behaviours[NAV_LAYER].active = false;
			behaviours[TURN_LAYER].active = false;
			Flytrap.rcon.out.println("done");
			on = false;
		}
		
		
		// was a turn in place target, finished turning
		if (behaviours[TURN_LAYER].active == true) {
			behaviours[TURN_LAYER].active = false;
		}
	}

}
