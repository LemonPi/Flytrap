package flytrap;

import lejos.hardware.Button;

public class EmergencyStop implements Runnable {
	public void run() {
		while (!Button.LEFT.isDown()) {
			try {
				Thread.sleep(100);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		System.exit(0);
	}
	public static void start() {
		Thread theThread = new Thread(new EmergencyStop());
		theThread.setDaemon(true);
		theThread.start();
		Runtime.getRuntime().addShutdownHook(new Thread(new ResetHook()));
	}
	static class ResetHook implements Runnable {
		public void run() {
			Engine.on = false;
			GetBall.open_claw();
			GetBall.lift_claw(GetBall.CLAW_RAISED);
		}
	}
}
