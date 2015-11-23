package flytrap;
import static flytrap.Engine.*;
import static java.lang.Math.*;

public class Turn {
	static final int TURN_SPEED = 100 / CYCLES_PER_SEC;	// degrees per second
	static int turn_size;
	static int to_turn;
	
	public static void turn_in_place() {
		Behaviour turn = behaviours[TURN_LAYER];
		turn.speed = 0;
		
		if (!turn.active) return;
		
		to_turn = wrap_angle((int)(target.angle - heading));
		// would be faster to turn in the other direction
		
		// turn until heading ~= targetted heading
		if (abs(to_turn) < HEADING_TOLERANCE) {
			Flytrap.rcon.out.println("dt");
			waypoint(TURN_LAYER);
			return;
		}
		
		// compare against initial turn size
		turn.turn = to_turn / turn_size * TURN_SPEED;
//		Flytrap.rcon.out.println("- turn" + to_turn + "/" + turn_size + "=>" + turn.turn);
		if (abs(turn.turn) < MIN_SPEED) {
			if (to_turn < 0) turn.turn = -MIN_SPEED;
			else turn.turn = MIN_SPEED;
		}
	}
}
