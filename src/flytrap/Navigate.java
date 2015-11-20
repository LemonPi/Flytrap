package flytrap;
import java.util.*;
import lejos.hardware.motor.*;
import static flytrap.Engine.*;

public class Navigate {
	static final double TURN_GAIN = 2;

	static double rho, alpha;	// rho is how far away from target, alpha is heading error

	public static void locate_target() {
		if (targets.isEmpty()) return;
		Target cur_target = targets.get(targets.size() - 1);
		double dx = cur_target.x - rx;
		double dy = cur_target.y - ry;
		double diff_theta = Math.toDegrees(Math.atan2(dy,dx));

		rho = Math.sqrt(dx*dx + dy*dy);
		// keep heading error within [-180,180]
		alpha = diff_theta - heading;
		if (alpha > 180) alpha -= 360;
		else if (alpha < 180) alpha += 360;
	}

	public static void navigate() {
		Behaviour nav = behaviours[NAV_LAYER];
		if (!nav.active || behaviours[TURN_LAYER].active) return;

		// reached target
		if (rho < TARGET_CLOSE_ENOUGH) {
			// don't care about turning or turned enough
			if (targets.get(targets.size() - 1).angle == ANY_HEADING ||
				Math.abs(targets.get(targets.size() - 1).angle - heading) < HEADING_TOLERANCE) {

				behaviours[TURN_LAYER].active = true;
				Turn.turn_size = (int)Math.abs(targets.get(targets.size() - 1).angle - heading);
				++process_cycles;
			}
			else {
				Flytrap.rcon.out.println("f");
				waypoint(NAV_LAYER);
			}
		}
		// large enough of angle difference to turn in place
		else if (alpha > HEADING_THRESHOLD_TURN_IN_PLACE) {
			Target in_place_target = new Target();
			in_place_target.type = TURN_LAYER;
			in_place_target.x = (int)rx;
			in_place_target.y = (int)ry;
			in_place_target.angle = (int)(alpha + heading);
			behaviours[TURN_LAYER].active = true;
			nav.active = false;

			Flytrap.rcon.out.println("t " + alpha);
		}
		// else navigate based on proportional error
		else {
			if (Math.abs(alpha) < HEADING_TOLERANCE) nav.turn = 0;
			else {
				nav.turn = (int) (alpha * TURN_GAIN);
			}
			// slow down closer to target
			if (rho < TARGET_CLOSE)
				nav.speed = (int)(TOP_SPEED * (rho/TARGET_CLOSE));
			else
				nav.speed = TOP_SPEED;
			// prevent stalling from going backwards
			if (nav.speed < Math.abs(nav.turn))
				nav.speed = Math.abs(nav.turn);
		}

	}
}
