package flytrap;

import java.util.*;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;

import static java.lang.Math.*;

import static flytrap.Engine.*;

public class AvoidBoundary {
	static final int NONE_ACTIVE = -1;
	/** index of the active boundary; can be NONE_ACTIVE if there are no nearby boundaries */
	static int active_boundary = NONE_ACTIVE;
	static List<Boundary> boundaries = new ArrayList<Boundary>();
	/** can stop tracking active boundary this far away */
	private static double BOUNDARY_FAR_ENOUGH = 100;
	private static double BOUNDARY_HEADING_CLEARENCE = 100;	// degrees between target and boundary before disregarding it
	private static double BOUNDARY_TOO_CLOSE = 300;
	/** corresponds to how wide it is */
	private static double BOUNDARY_TOLERANCE = PI*0.5;
	private static double BOUNDARY_FOLLOW_TOLERANCE = PI*0.04;
	private static double EXISTENTIAL_THREAT = 0.3*BOUNDARY_TOO_CLOSE;
	/** how hard to turn away from obstacle; adjustable */
	private static int BOUND_TURN = 100/CYCLES_PER_SEC;
	
	/* sensory */
	private static double BOUNDARY_SENSOR_MIN = 50; // 5cm
	private static double BOUNDARY_SENSOR_MAX = 500; // 50cm
	private static int BOUNDARY_SENSOR_DEPTH = 30;  // offset of sensor that needs to be subtracted from center
	// radius of detected obstacle
	private static double BOUNDARY_SENSOR_RADIUS = 40;
	private static SensorMode distanceSensorMode;
	private static float[] distanceSamples;
	static {
		distanceSensorMode = new EV3IRSensor(SensorPort.S3).getDistanceMode();
		distanceSamples = new float[distanceSensorMode.sampleSize()];
	}

	public static void avoid_boundary() {
		sense_boundary();
		Behaviour bound = behaviours[BOUNDARY_LAYER];
		// deactivate boundary avoidance when moved away far enough
		if (active_boundary != NONE_ACTIVE &&
				((abs(wrap_angle(deg(boundaries.get(active_boundary).theta) - Navigate.alpha)) >  BOUNDARY_HEADING_CLEARENCE &&// not obstructing the path to goal
						boundaries.get(active_boundary).distance > BOUNDARY_FAR_ENOUGH) ||
				boundaries.get(active_boundary).distance > Navigate.rho)) {
			Flytrap.rcon.out.println("BF");
			Boundary boundary = boundaries.get(active_boundary);
			Flytrap.sendBoundary((int)boundary.x, (int)boundary.y, (int)boundary.r, 0);
			active_boundary = NONE_ACTIVE;
			bound.active = false;
		}
		for (int b = 0; b < boundaries.size(); b++) {
			Boundary boundary = boundaries.get(b);
			// check distance to boundary
			double diff_x = boundary.x - rx;
			double diff_y = boundary.y - ry;
			double theta = rad(heading);
			
			boundary.prev_distance = boundary.distance;
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
		// don't avoid boundary if you're turning in place 
		if (behaviours[TURN_LAYER].active || 
			targets.get(targets.size() - 1).type == TARGET_GET || 
				behaviours[GET_LAYER].active) return;
		
		// take care of the case when all boundaries are inactive
		double max_threat = 0;
		int prev_boundary = active_boundary;

		for (int b = 0; b < boundaries.size(); b++) {
			Boundary boundary = boundaries.get(b);
			if (boundary.threat > max_threat) {
				active_boundary = b;
				max_threat = boundary.threat;
			}		
		}

		
		if (active_boundary != prev_boundary) {
			if (prev_boundary != NONE_ACTIVE) {
				Boundary boundary = boundaries.get(prev_boundary);
				Flytrap.sendBoundary((int)boundary.x, (int)boundary.y, (int)boundary.r, 0);
			}
			if (active_boundary != NONE_ACTIVE) {
				Boundary boundary = boundaries.get(active_boundary);
				Flytrap.sendBoundary((int)boundary.x, (int)boundary.y, (int)boundary.r, 1);
			}
		}
		// if closer to target than boundary
		if (active_boundary == NONE_ACTIVE || Navigate.rho < boundaries.get(active_boundary).distance)
			return;
		
		// // if there is an active boundary and enough of a threat
		if (active_boundary != NONE_ACTIVE) {
			Boundary boundary = boundaries.get(active_boundary);
			if (boundary.threat >= EXISTENTIAL_THREAT) bound.active = true;

			bound.speed = (int) (TOP_SPEED * 0.5);
			// want to keep boundary at +- 90 degrees to hug around it
			// if boundary_heading_error > 0, robot is on left of boundary
			if (boundary.theta > 0) {
				if (abs(boundary.theta - PI/2) < BOUNDARY_FOLLOW_TOLERANCE) bound.turn = 0;
				else if (boundary.theta > PI/2) bound.turn = BOUND_TURN;
				else bound.turn = -BOUND_TURN;
			}
			else {
				if (abs(boundary.theta + PI/2) < BOUNDARY_FOLLOW_TOLERANCE) bound.turn = 0;
				else if (boundary.theta < -PI/2) bound.turn = -BOUND_TURN;
				else bound.turn = BOUND_TURN;
			}
			
			//Flytrap.rcon.out.println("- " + boundary.x + " " + boundary.y);
		}
	}
	public static void add_boundary(int x, int y, int r) {
		for (int b = 0; b < boundaries.size(); b++) {
			Boundary boundary = boundaries.get(b);
			double diff_x = x - boundary.x;
			double diff_y = y - boundary.y;
			double distFromExisting = sqrt(sq(diff_x)+sq(diff_y));
			if (distFromExisting < boundary.r * 4) { // inside the existing bound
				return;
			}
//			else if (distFromExisting < (boundary.r + r) * 3) { 	// close enough to merge
//				if (boundary.r > BOUNDARY_SENSOR_RADIUS * 6)
//					return;
//				// how close the new center will be to the old center (for an existing large radius, new center will be closer)
//				++boundary.merged;
//				double center_weighting = 1 / boundary.merged;
////				double center_weighting = r / (r + boundary.r);
//				boundary.x += center_weighting * diff_x;
//				boundary.y += center_weighting * diff_y;
//				boundary.r = (distFromExisting + boundary.r + r) / 2; // enclose the 2 boundaries
//				Flytrap.sendBoundary((int)boundary.x, (int)boundary.y, (int)boundary.r, 0);
//				Flytrap.rcon.out.println("- center_weighting " + center_weighting + " diff " + diff_x +
//						":" + diff_y + " distFromExisting " + distFromExisting + " boundary.r " + boundary.r + " r " + r);
//				return;
//			}
		}
		// else is a new boundary
		Boundary boundary = new Boundary();
		boundary.x = x;
		boundary.y = y;
		boundary.r = r;
		boundaries.add(boundary);
		Flytrap.sendBoundary(x, y, r, 0);		
	}
	static double sense_boundary() {
//		if (boundaries.size() > 0) return;
		distanceSensorMode.fetchSample(distanceSamples, 0);
		double distance = distanceSamples[0]*10 - BOUNDARY_SENSOR_DEPTH; // to mm
		if (distance < BOUNDARY_SENSOR_MIN || distance > BOUNDARY_SENSOR_MAX) return distance;
		// sensor shouldn't give readings less than 50 mm or greater than 500 mm
		double theta = rad(heading);
		double obsx = distance*cos(theta) + rx;
		double obsy = distance*sin(theta) + ry;

		// add boundary will take of checking if it can be merged with an existing boundary
		add_boundary((int)obsx, (int)obsy, (int)BOUNDARY_SENSOR_RADIUS);
		return distance;
	}
	public static void init() {
		// empty since this is just needed to call static initializers
	}
}
