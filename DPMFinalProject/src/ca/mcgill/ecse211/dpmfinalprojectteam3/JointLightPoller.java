package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.robotics.SampleProvider;

/**
 * Created this class to synchronize the readings from the left and right light
 * sensors. This thread provides samples for the light localization class and
 * odometry correction class for detecting lines, we need to synchronize these
 * readings in order to make the readings of both sensors more accurate,
 * especially in cases where both of the light sensors detect a line for a
 * period of time.
 */
public class JointLightPoller extends Thread {

	/** The leftprovider. */
	private SampleProvider leftprovider;

	/** The rightprovider. */
	private SampleProvider rightprovider;

	/** The sample. */
	private float[] sample;

	/** The left light val. */
	private double leftLightVal = 0;

	/** The right light val. */
	private double rightLightVal = 0;

	/** The left last light val. */
	// private double leftLastLightVal = 0;

	/** The right last light val. */
	// private double rightLastLightVal = 0;

	/** The on. */
	public boolean on;

	/** The light data. */
	private double[] lightData = new double[6];

	/**
	 * Instantiates a new joint light poller.
	 *
	 * @param leftprovider
	 *            the leftprovider
	 * @param rightprovider
	 *            the rightprovider
	 */
	public JointLightPoller(SampleProvider leftprovider, SampleProvider rightprovider) {
		this.leftprovider = leftprovider;
		this.rightprovider = rightprovider;
		on = false;

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		sample = new float[2];
		while (true) {
			if (on) {
				leftprovider.fetchSample(sample, 0); // acquire data
				rightprovider.fetchSample(sample, 1);
				leftLightVal = (double) (sample[0]);// extract from buffer, cast to int
				rightLightVal = (double) (sample[1]);
				synchronized (this) {
					lightData[0] = leftLightVal;
					lightData[1] = rightLightVal;
				}

				try {
					Thread.sleep(11);
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
	 * Gets the light values from both sensors, synchronized so they cant be
	 * changed.
	 *
	 * @return the values
	 */
	public double[] getValues() {
		double[] result;
		synchronized (this) {
			result = lightData;
		}

		return result;
	}

	/**
	 * Gets the light value in red mode from the left sensor, synchronized with the
	 * right light sensor.
	 *
	 * @return the left value
	 */
	public double getLeftValue() {
		double result;
		synchronized (this) {
			result = lightData[0];
		}

		return result;
	}

	/**
	 * Gets the light value in red mode from the right sensor, synchronized with the
	 * left light sensor.
	 *
	 * @return the right value
	 */
	public double getRightValue() {
		double result;
		synchronized (this) {
			result = lightData[1];
		}

		return result;
	}

	/**
	 * turns the thread on.
	 */
	public void on() {
		this.on = true;

	}

	/**
	 * turns the thread off.
	 */
	public void off() {
		this.on = false;
	}

}
