package flytrap;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Params {
	public static final int MOLERAT = 0;
	public static final int VENUS = 1;
	
	
//	// Molerat bot
	public static final int platform = MOLERAT;
	public static final double MM_PER_DEGREE = 134.0/360;
	public static final double BASE_WIDTH = 124.406;
	
	/* sensory */
	static double BOUNDARY_SENSOR_MIN = 50; // 5cm
	static double BOUNDARY_SENSOR_MAX = 500; // 50cm
	static int BOUNDARY_SENSOR_DEPTH = 30;  // offset of sensor that needs to be subtracted from center
	static int DISTANCE_SENSOR_SCALE = 10;	// units in cm
	static SampleProvider distanceSensorMode;
	static float[] distanceSamples;
	static SampleProvider gyroSensor;
	static float[] gyroSamples;
	static {
		distanceSensorMode = new EV3IRSensor(SensorPort.S3).getDistanceMode();
		distanceSamples = new float[distanceSensorMode.sampleSize()];
		gyroSensor = null;
		gyroSamples = null;
	}
	
	
	
	// Venus bot
//	public static final int platform = VENUS;
//	public static final double MM_PER_DEGREE = 173.5/360.0; // 3470 / 20.0 / 360
//	public static final double BASE_WIDTH = 109.2263; // 109.2263
//	static double BOUNDARY_SENSOR_MIN = 10;
//	static double BOUNDARY_SENSOR_MAX = 2500;
//	static int BOUNDARY_SENSOR_DEPTH = 22;  // offset of sensor that needs to be subtracted from center
//	static int DISTANCE_SENSOR_SCALE = 1000;	// units in m
//	static EV3GyroSensor gyroSense;
//	static SampleProvider distanceSensorMode;
//	static float[] distanceSamples;
//	static SampleProvider gyroSensor;
//	static float[] gyroSamples;
//	static {
//		distanceSensorMode = new EV3UltrasonicSensor(SensorPort.S3).getDistanceMode();
//		distanceSamples = new float[distanceSensorMode.sampleSize()];
//		gyroSense = new EV3GyroSensor(SensorPort.S2);
//		gyroSensor = gyroSense.getAngleMode();
//		gyroSamples = new float[gyroSensor.sampleSize()];
//	}
}
