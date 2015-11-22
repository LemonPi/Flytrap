package flytrap;

import java.util.*;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;

import static java.lang.Math.*;

import static flytrap.Engine.*;

public class AvoidBoundary {
	private static final int NONE_ACTIVE = -1;
	/** index of the active boundary; can be NONE_ACTIVE if there are no nearby boundaries */
	private static int active_boundary = NONE_ACTIVE;
	private static List<Boundary> boundaries = new ArrayList<Boundary>();
	/** can stop tracking active boundary this far away */
	private static double BOUNDARY_FAR_ENOUGH = 100;
	private static double BOUNDARY_TOO_CLOSE = 200;
	/** corresponds to how wide it is */
	private static double BOUNDARY_TOLERANCE = PI*0.5;
	private static double EXISTENTIAL_THREAT = 0.3*BOUNDARY_TOO_CLOSE;
	/** how hard to turn away from obstacle; adjustable */
	private static int BOUND_TURN = 20;
	
	/* sensory */
	private static double BOUNDARY_SENSOR_MIN = 50; // 5cm
	private static double BOUNDARY_SENSOR_MAX = 500; // 50cm
	// 2cm - radius of detected obstacle
	private static double BOUNDARY_SENSOR_RADIUS = 20;
	private static SensorMode distanceSensorMode;
	private static float[] distanceSamples;
	static {
		distanceSensorMode = new EV3IRSensor(SensorPort.S3).getDistanceMode();
		distanceSamples = new float[distanceSensorMode.sampleSize()];
	}

	public static void avoid_boundary() {
		sense_boundary();
		Behaviour bound = behaviours[BOUNDARY_LAYER];
		// deactivate boundary avoidance when far enough
		if (active_boundary != NONE_ACTIVE &&
				boundaries.get(active_boundary).distance > BOUNDARY_FAR_ENOUGH ) {
			active_boundary = NONE_ACTIVE;
			bound.active = false;
		}
		for (int b = 0; b < boundaries.size(); b++) {
			Boundary boundary = boundaries.get(b);
			// check distance to boundary
			double diff_x = boundary.x - rx;
			double diff_y = boundary.y - ry;
			double theta = rad(heading);
			// approximate each boundary as circle, from center to point - radius
			boundary.distance = sqrt(sq(diff_x) + sq(diff_y)) - boundary.r;
			if (boundary.distance < 0) { boundary.distance = 0; boundary.threat = EXISTENTIAL_THREAT; }
			// compare this with theta to see if collision likely
			boundary.theta = atan2(diff_y, diff_x) - theta;
			if (boundary.theta > PI) boundary.theta -= 2*PI;
			else if (boundary.theta < -PI) boundary.theta += 2*PI;

			if (boundary.distance < BOUNDARY_TOO_CLOSE &&
				(abs(boundary.theta) < BOUNDARY_TOLERANCE)) {

				// high threat comes from being closer and a straight hit
				boundary.threat = (BOUNDARY_TOO_CLOSE - boundary.distance) * 
									(BOUNDARY_TOLERANCE - abs(boundary.theta)) / BOUNDARY_TOLERANCE;
		
			}
			// no threat, either angle not a concern or too far away
			else boundary.threat = 0;

		}
		// don't avoid boundary if you're turning in place or close to target
		if (behaviours[TURN_LAYER].active || Navigate.rho < TARGET_CLOSE_ENOUGH || 
			targets.get(targets.size() - 1).type == TARGET_GET || 
				behaviours[GET_LAYER].active) return;
		
		// take care of the case when all boundaries are inactive
		double max_threat = 0;

		for (int b = 0; b < boundaries.size(); b++) {
			Boundary boundary = boundaries.get(b);
			if (boundary.threat > max_threat) {
				active_boundary = b;
				max_threat = boundary.threat;
			}		
		}

		// // if there is an active boundary and enough of a threat
		if (active_boundary != NONE_ACTIVE) {
			Boundary boundary = boundaries.get(active_boundary);
			if (boundary.threat >= EXISTENTIAL_THREAT) bound.active = true;

			bound.speed = (int) (TOP_SPEED * 0.5);
			// want to keep boundary at +- 90 degrees to hug around it
			// if boundary_heading_error > 0, robot is on left of boundary
			if (boundary.theta > 0) {
				if (boundary.theta > 2*PI) bound.turn = BOUND_TURN;
				else bound.turn = -BOUND_TURN;
			}
			else {
				if (boundary.theta < -2*PI) bound.turn = -BOUND_TURN;
				else bound.turn = BOUND_TURN;
			}
		}
	}
	private static void sense_boundary() {
		distanceSensorMode.fetchSample(distanceSamples, 0);
		double distance = distanceSamples[0]*10; // to mm
		if (distance < BOUNDARY_SENSOR_MIN || distance > BOUNDARY_SENSOR_MAX) return;
		// sensor shouldn't give readings less than 50 mm or greater than 500 mm
		double theta = rad(heading);
		double obsx = distance*cos(theta) + rx;
		double obsy = distance*sin(theta) + ry;
		for (int b = 0; b < boundaries.size(); b++) {
			Boundary boundary = boundaries.get(b);
			double diff_x = boundary.x - obsx;
			double diff_y = boundary.y - obsy;
			double distFromExisting = sq(diff_x)+sq(diff_y);
			if (distFromExisting < sq(boundary.r)) { // inside the existing bound
				// need to do kalman from here
				return;
			}
		}
		// not an existing boundary, add it
		Boundary boundary = new Boundary();
		boundary.x = obsx;
		boundary.y = obsy;
		boundary.r = BOUNDARY_SENSOR_RADIUS;
		boundaries.add(boundary);
		Flytrap.sendPoint(BOUNDARY_POS_LAYER, (int)obsx, (int)obsy);
	}
}
