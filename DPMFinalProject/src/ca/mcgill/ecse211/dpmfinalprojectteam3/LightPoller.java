package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

// 
/**
 * Class used for sampling data for the light sensors, particularly the color
 * sensor that will be used for color detection. The other two sensors for line
 * detection will use the JointLightPoller class to synchronize their data
 * better.
 */
public class LightPoller extends Thread {

	/** The sensor. */
	private EV3ColorSensor sensor;

	/** The provider. */
	private SampleProvider provider;

	/** The sample. */
	private float[] sample;

	/** The light val. */
	private double lightVal = 0;
<<<<<<< Updated upstream
=======
	private static double[] colorValues = new double[3];
>>>>>>> Stashed changes

	/** The last light val. */
	private double lastLightVal = 0;

	/** The on. */
	public boolean on;

	/**
	 * Instantiates a new light poller.
	 *
	 * @param sensor
	 *            the sensor
	 * @param provider
	 *            the provider
	 */
	public LightPoller(EV3ColorSensor sensor, SampleProvider provider) {
		this.sensor = sensor;
		this.provider = provider;
		on = false;

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		sample = new float[1];
		while (true) {
			if (on) {
				synchronized (this) {
					lastLightVal = lightVal;
					provider.fetchSample(sample, 0); // acquire data
					lightVal = (double) (sample[0]); // extract from buffer, cast to int
				}
				try {
					Thread.sleep(50);
				} catch (Exception e) {
				} // Poor man's timed sampling
			} else {
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
				}
			}
		}
	}

	/**
	 * Gets the change in light.
	 *
	 * @return the change in light
	 */
	public double getChangeInLight() {
		double result;
		synchronized (this) {
			result = (lightVal - lastLightVal) / lastLightVal;
		}
		return result;
	}

	/**
	 * Gets the light val.
	 *
	 * @return the light val
	 */
	public double getLightVal() {
		double result;
		synchronized (this) {
			result = lightVal;
		}

		return result;
	}

	/**
	 * Gets the last light val.
	 *
	 * @return the last light val
	 */
	public double getLastLightVal() {
		double result;
		synchronized (this) {
			result = lastLightVal;
		}
		return result;
	}

	/**
	 * turn thread on.
	 */
	public void on() {
		on = true;
	}

	/**
	 * turn thread off.
	 */
	public void off() {
		on = false;
	}
<<<<<<< Updated upstream
=======

	public static boolean checkColors() {
		colorValues[0] = colorValues[0] * 100;
		colorValues[1] = colorValues[1] * 100;
		colorValues[2] = colorValues[2] * 100;
		System.out.println(colorValues[0]);
		System.out.println(colorValues[1]);
		System.out.println(colorValues[2]);

			return (colorValues[0] < FinalProject.RGBColors[0] + FinalProject.EPSILON
					&& colorValues[0] > FinalProject.RGBColors[0] - FinalProject.EPSILON)
					&& (colorValues[1] < FinalProject.RGBColors[1] + FinalProject.EPSILON
							&& colorValues[1] > FinalProject.RGBColors[1] - FinalProject.EPSILON)
					&& (colorValues[2] < FinalProject.RGBColors[2] + FinalProject.EPSILON
							&& colorValues[2] > FinalProject.RGBColors[2] - FinalProject.EPSILON);

		
	}
>>>>>>> Stashed changes
}
