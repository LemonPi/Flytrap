package flytrap;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import static flytrap.Engine.*;
import static java.lang.Math.*;

public class GetBall {
	static final int BALL_LESS = 0;
	static final int JUST_GOT_BALL = 1;
	static final int CAUGHT_BALL = 5; 
	static final int RELEASED_BALL = 10;
	static final int SECURED_BALL = 20;
	static int ball_status = BALL_LESS;
	static final int GET_TURN = 10;
	static final int GET_SPEED = 444444;
	static final int THETA_TOLERANCE = 3;
	private static float[] touchsample = new float[1];
	private static EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S2);
	//private static final int get_initial_distance;

	static void get_ball() {
		Behaviour get = behaviours[GET_LAYER];
		Target hopper = null; // FIXME
		if (!get.active) return;
		if (active_behaviour == GET_LAYER) {
			// todo log
		}
		if (ball_status < CAUGHT_BALL) {
			if (caught_ball()) ++ball_status;
			get.speed = GET_SPEED;
			if (abs(hopper.angle) < THETA_TOLERANCE) get.turn = 0;
			else if (hopper.angle < 0) get.turn = -GET_TURN;
			else get.turn = GET_TURN;
			if (ball_status == CAUGHT_BALL) {
				// Correct.correct_to_hopper()
				close_gate();
				//hard_break(GET_LAYER);
				//get_initial_distance = tot_distance;
			}
		}
	}
	/** returns true if we're touching the ball hopper */
	private static boolean caught_ball() {
		touch.fetchSample(touchsample, 0);
		return touchsample[0] > 0.5;
	}
	private static void close_gate() {
		// TODO
	}
}
