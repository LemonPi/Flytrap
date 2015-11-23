package flytrap;
import static flytrap.Engine.*;
import static java.lang.Math.*;
import static flytrap.AvoidBoundary.*;

public class Navigate {
	static final double TURN_GAIN = 0.15;

	static double rho, alpha, last_rho;	// rho is how far away from target, alpha is heading error

	public static void locate_target() {
		if (targets.isEmpty()) return;
		double dx = target.x - rx;
		double dy = target.y - ry;
		double diff_theta = Math.toDegrees(Math.atan2(dy,dx));

		rho = Math.sqrt(dx*dx + dy*dy);
		// keep heading error within [-180,180]
		alpha = wrap_angle(diff_theta - heading);
	}

	public static void navigate() {
		Behaviour nav = behaviours[NAV_LAYER];
		if (!nav.active || behaviours[TURN_LAYER].active) return;

		// reached target
		if ((rho < TARGET_IMMEDIATE) || 
				(rho < TARGET_CLOSE_ENOUGH && rho > last_rho)) {	// can't get any closer
			nav.active = false;
			Flytrap.rcon.out.println('c');
			
			double beta = wrap_angle(target.angle - heading);
			// needs turning at target
			if (target.angle != ANY_HEADING && allowed_behaviour(TURN_LAYER) &&
				Math.abs(beta) > HEADING_TOLERANCE) {

				behaviours[TURN_LAYER].active = true;
				hard_break(NAV_LAYER, 3);
				Turn.turn_size = (int)Math.abs(beta);
				Flytrap.rcon.out.println("t " + Turn.turn_size);
				
				++process_cycles;
			}
			// don't need turning
			else {
				Flytrap.rcon.out.println("f");
				waypoint(NAV_LAYER);
			}
		}

		// large enough of angle difference to turn in place
		else if ((active_boundary == NONE_ACTIVE || // no active boundaries
				 		(boundaries.get(active_boundary).distance > rho &&	// target is closer than boundary
				 		abs(boundaries.get(active_boundary).theta - rad(alpha)) > 0.3)) &&
				  rho >  TARGET_IMMEDIATE &&
				  abs(alpha) > HEADING_THRESHOLD_TURN_IN_PLACE && 
				  allowed_behaviour(TURN_LAYER)) {

			add_target((int)rx, (int)ry, (int)(alpha + heading), TURN_LAYER);
			
			Turn.turn_size = (int)abs(alpha);
			hard_break(NAV_LAYER, 3);
			behaviours[TURN_LAYER].active = true;
			nav.active = false;

			Flytrap.rcon.out.println("t " + (int)alpha + " to " + target.angle);
		}
		// else navigate based on proportional error
		else {
			if (Math.abs(alpha) < HEADING_TOLERANCE) nav.turn = 0;
			else {
				nav.turn = (int) (alpha * TURN_GAIN);
			}
			// slow down closer to target
			if (rho < TARGET_CLOSE) {
				nav.speed = (int)(TOP_SPEED * (rho/TARGET_CLOSE));
				if (nav.speed < MIN_SPEED) nav.speed = MIN_SPEED;
			}
			else
				nav.speed = TOP_SPEED;
			// prevent stalling from going backwards
			if (nav.speed < Math.abs(nav.turn))
				nav.speed = Math.abs(nav.turn);
		}
		last_rho = rho;
	}
}
