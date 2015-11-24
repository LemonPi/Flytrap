package flytrap;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import static flytrap.Engine.*;
import static java.lang.Math.*;

public class GetBall {
	static final int IDLE_STATE = 0;
	static final int SWIVEL_LEFT = 1;
	static final int SWIVEL_RIGHT = 2;
	static final int SWIVEL_TARGET = 3;
	static final int FORWARD_TARGET = 4;
	static final int GRAB_BALL = 5;
	static final int BACK_AWAY = 6;
	static final int GET_BALL_BOUND = 100; // 10 cm
	static final int GET_BALL_SWIVEL = 30; // degrees left, right
	static final int GET_BALL_CRAWL = 20; // 2 cm
	static final int GET_BALL_CLOSE_ENOUGH = 30; //3 cm
	static final int GET_BALL_CRAWL_SPEED = 2; // forward speed
	static final int GET_BALL_TOO_FAR_TOLERANCE = 10; // go back into swivel if 1cm farther than initial distance
	static int ball_status = IDLE_STATE;
	static double minDist = 9999;
	static double minAngle = 0;
	static double distRemaining = 0;
	static NXTRegulatedMotor liftMotor = Motor.C;
	static NXTRegulatedMotor clawMotor = Motor.D;
	
	static final int NUM_CLAW_POS = 4;
	static final int DEGREES_PER_FULL_RANGE = 330;
	static final int CLAW_RAISED = 0;
	static final int CLAW_LOWERED = NUM_CLAW_POS - 1;

	static void lift_claw(int pos) {
		liftMotor.rotateTo(pos * (DEGREES_PER_FULL_RANGE/(NUM_CLAW_POS - 1)));
	}
	static void open_claw() {
		clawMotor.rotateTo(0);
	}
	static void close_claw() {
		clawMotor.rotateTo(-90);
	}
	
	private static boolean do_turn(Behaviour behaviour, double target) {
		int to_turn = wrap_angle((int)(target - heading));
		// would be faster to turn in the other direction

		// turn until heading ~= targeted heading
		if (abs(to_turn) < HEADING_TOLERANCE) {
			return true;
		}
		behaviour.speed = 0;
		behaviour.turn = to_turn > 0? 2: -2;
		return false;
	}

	static void get_ball() {
		if (target == null || !(target.type == GET_LAYER || target.type == PUT_LAYER)) return;
		Behaviour behaviour = Engine.behaviours[GET_LAYER];
		double dx = target.x - rx;
		double dy = target.y - ry;
		if (dx*dx + dy*dy > GET_BALL_BOUND*GET_BALL_BOUND && ball_status == IDLE_STATE) {
			behaviour.active = false;
			return;
		} else {
			behaviour.active = true;
			if (ball_status == IDLE_STATE) ball_status = SWIVEL_LEFT;
		}
		double angleToTarg = deg(atan2(dy, dx));
		double angleLeft = wrap_angle(angleToTarg + GET_BALL_SWIVEL);
		double angleRight = wrap_angle(angleToTarg - GET_BALL_SWIVEL);
		switch (ball_status) {
		case SWIVEL_LEFT: {
			if (do_turn(behaviour, angleLeft)) {
				ball_status = SWIVEL_RIGHT;
				behaviour.turn = behaviour.speed = 0;
				minDist = 99999999;
			}
			break;
		}
		case SWIVEL_RIGHT: {
			if (do_turn(behaviour, angleRight)) {
				ball_status = SWIVEL_TARGET;
				behaviour.turn = behaviour.speed = 0;
			} else {
				double dist = AvoidBoundary.sense_object();
				double sensedDx = dist*cos(rad(heading));
				double sensedDy = dist*sin(rad(heading));
				// compare offset info from sensor with expected offset of ball
				double diffFromActual = abs(sensedDx - dx) + abs(sensedDy - dy);
				if (diffFromActual < minDist) {
					minDist = diffFromActual;
					minAngle = heading;
				}
			}
			break;
		}
		case SWIVEL_TARGET:{
			if (do_turn(behaviour, minAngle)) {
				ball_status = FORWARD_TARGET;
				distRemaining = AvoidBoundary.sense_object();
				behaviour.turn = behaviour.speed = 0;
			}
			break;
		}
		case FORWARD_TARGET:{
			double dist = AvoidBoundary.sense_object();
			Flytrap.rcon.out.println("- in forward target - dist " + dist + " original " + distRemaining + " minAngle " + minAngle);
			if (dist < GET_BALL_CLOSE_ENOUGH) {
				ball_status = GRAB_BALL;
				behaviour.turn = behaviour.speed = 0;
			} else if (dist > distRemaining - GET_BALL_TOO_FAR_TOLERANCE){
				ball_status = SWIVEL_LEFT;
				behaviour.turn = behaviour.speed = 0;
			} else{
				behaviour.speed = GET_BALL_CRAWL_SPEED;
				behaviour.turn = 0;
			}
			break;
		}
		case GRAB_BALL: {
			if (target.type == PUT_LAYER) {
				do_put_ball();
			} else {
				do_grab_ball();
			}
			ball_status = BACK_AWAY;
			break;
		}
		case BACK_AWAY: {
			double curDist = dx*dx + dy*dy;
			if (curDist >= sq(GET_BALL_BOUND) + 10) {
				ball_status = IDLE_STATE;
				behaviour.active = false;
				behaviour.turn = behaviour.speed = 0;
				// Done, exit
				waypoint(GET_LAYER);
			} else {
				behaviour.turn = 0;
				behaviour.speed = -GET_BALL_CRAWL_SPEED;
			}
			break;
		}
		default: {
			throw new RuntimeException("Invalid state in get ball: " + ball_status);
		}
		}
		Flytrap.rcon.out.println("- GetBall: " + ball_status + ":" + angleLeft + ":" + angleRight);
	}
	private static void do_grab_ball() {
		lift_claw(CLAW_LOWERED);
		close_claw();
		lift_claw(CLAW_RAISED);
	}
	private static void do_put_ball() {
		open_claw();
	}
}
