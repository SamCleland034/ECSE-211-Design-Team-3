package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

// TODO: Auto-generated Javadoc
// 
/**
 * Class used for sampling data for the light sensors, particularly the color
 * sensor that will be used for color detection. The other two sensors for line
 * detection will use the JointLightPoller class to synchronize their data with
 * each other. Will be terminated once the robot has found the flag. Using RGB
 * mode for this class.
 */
public class LightPoller extends Thread {

	/** The sensor. */
	private EV3ColorSensor sensor;

	/** The provider. */
	private SampleProvider provider;

	/** The array data will be in to. */
	private float[] sample;

	/** The light value that is read. */
	private double lightVal = 0;

	/** The color values. */
	private double[] colorValues = new double[3];

	/** The last light value read. */
	private double lastLightVal = 0;

	/** boolean to indicate if the thread is on. */
	public boolean on;

	private Navigation gps;

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
		sample = new float[provider.sampleSize()];
		while (!gps.hasFlag) {
			if (on) {
				synchronized (this) {

					provider.fetchSample(sample, 0); // acquire data
					colorValues[0] = (double) 100 * sample[0];// extract from buffer, cast to int
					colorValues[1] = (double) 100 * sample[1];
					colorValues[2] = (double) 100 * sample[2];
					System.out.print("R:" + colorValues[0]);
					System.out.print("  G:" + colorValues[1]);
					System.out.println("  B:" + colorValues[2]);
				}
				try {
					Thread.sleep(30);
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
	public double[] getLightVal() {
		double[] result;
		synchronized (this) {
			result = colorValues;
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

	/**
	 * Check for white color
	 *
	 * @return true, if reading white color
	 */
	public boolean checkWhite() {
		boolean result = false;
		synchronized (this) {
			result = colorValues[0] > 0.45 && colorValues[1] > 0.45 && colorValues[2] > 0.098;
		}
		return result;
	}

	/**
	 * Checks for red color
	 *
	 * @return true, if reading red color
	 */
	public boolean checkRed() {
		boolean result = false;
		synchronized (this) {
			result = (colorValues[0] > colorValues[1] && colorValues[0] > colorValues[2] && colorValues[0] < 9.5);
		}
		return result;
	}

	/**
	 * Checks for blue color
	 *
	 * @return true, if reading blue color
	 */
	public boolean checkBlue() {
		boolean result = false;
		synchronized (this) {
			result = (colorValues[0] < colorValues[2] && colorValues[1] >= colorValues[2])
					|| (colorValues[0] < colorValues[2] && colorValues[1] <= colorValues[2]);
		}
		return result;
	}

	/**
	 * Checks for yellow color.
	 *
	 * @return true, if successful
	 */
	public boolean checkYellow() {
		boolean result = false;
		synchronized (this) {
			result = colorValues[0] > colorValues[1] && colorValues[1] > colorValues[2];
		}
		return result;
	}

	/**
	 * Method called from the navigation class when in flagsearch mode to read
	 * colors in from this poller. Using synchronize block so we get accurate values
	 * from 1 sampling interval.
	 *
	 * @param correctColor
	 *            the correct color that we are looking for, int from 1-4 depending
	 *            on what was passed in
	 * @return true, if it finds the color that was passed in.
	 */
	public boolean checkColors(int correctColor) {
		boolean result = false;
		synchronized (this) {
			if (correctColor == 1)
				result = checkRed();
			else if (correctColor == 2)
				result = checkBlue();
			else if (correctColor == 3)
				result = checkYellow();
			else if (correctColor == 4)
				result = checkWhite();
		}
		return result;
	}

	public void setNavigation(Navigation gps) {
		this.gps = gps;

	}
}
