package flytrap;
import Behaviour.*;
import lejos.hardware.motor.*;

public class Engine {
	// constants 
	public static final double KP = 1.194;
	public static final double KI = 1.2;
	public static final double KD = 0.005;
	public static final int FWD = 1;
	public static final int BWD = -1;
	public static final double MM_PER_DEGREE = ;
	public static final double BASE_WIDTH = ;

	public static final int NUM_BEHAVIOUR = 4;
	public static final int BOUNDARY_LAYER = 0;
	public static final int GET_LAYER = 1;	// gets activated when near ball dispensers without ball
	public static final int PUT_LAYER = 2;	// gets activated when near ball repostitory with ball
	public static final int NAV_LAYER = 3;	// otherwise on
	// internal state
	NXTRegulatedMotor motor_l = Motor.A;
	NXTRegulatedMotor motor_r = Motor.B;

	// target speeds (angles per cycle)
	int target_l = 0;
	int target_r = 0;
	// instantaneous speeds
	int instant_speed_l = 0;
	int instant_speed_r = 0;

	// PID related stuff
	int prev_l, prev_r;
	double integral_l, integral_r;

	Behaviour[] behaviours = new Behaviour[NUM_BEHAVIOUR];
	int active_behaviour = 0;

	double rx,ry,heading;	// heading is in degrees [-180,180]

	List<int[]> targets = new ArrayList<int[]>();


	public static <T extends Comparable<T>> T clamp(T val, T min, T max) {
	    if (val.compareTo(min) < 0) return min;
	    else if (val.compareTo(max) > 0) return max;
	    else return val;
	}
	public static double rad(double deg) {
		return Math.toRadians(deg);
	}

	public static void pid_control(int sl, int sr) {
		int err_l = target_l - sl;
		int err_r = target_r - sr;

		// derivative on measurement to fix derivative kick
		int input_change_l = (tl - prev_l);
		int input_change_r = (tr - prev_r);

		// update state
		prev_l = tl;
		prev_r = tr;

		// integral
		integral_l += KI * error_l;
		integral_r += KI * error_r;
		clamp(integral_l, 0, 255);
		clamp(integral_r, 0, 255);

		out_l = KP*err_l + integral_l - KD*input_change_l;
		out_r = KP*err_r + integral_r - KD*input_change_r;

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
		if (heading > 180) heading -= 360;
		else if (heading < 180) heading += 360;
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
				motor_control(b);
				active_behaviour = b;
				return;
			}
		}
	}

	public static boolean go() {

		odometry();

		// process behaviours


		pid_control(instant_speed_l, instant_speed_r);
		motor_l.setSpeed(out_l);
		motor_r.setSpeed(out_r);
		return true;
	}
}
