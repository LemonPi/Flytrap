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
	}
}
