package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.robotics.SampleProvider;

public class JointLightPoller extends Thread {
	private SampleProvider leftprovider;
	private SampleProvider rightprovider;
	private float[] sample;
	private double leftLightVal = 0;
	private double rightLightVal = 0;
	private double leftLastLightVal = 0;
	private double rightLastLightVal = 0;
	public boolean on;
	private double[] lightData = new double[6];

	public JointLightPoller(SampleProvider leftprovider, SampleProvider rightprovider) {
		this.leftprovider = leftprovider;
		this.rightprovider = rightprovider;
		on = false;

	}

	public void run() {
		sample = new float[2];
		while (true) {
			if (on) {

				// leftLastLightVal = leftLightVal;
				// rightLastLightVal = rightLightVal;
				leftprovider.fetchSample(sample, 0); // acquire data
				rightprovider.fetchSample(sample, 1);
				leftLightVal = (double) (sample[0]);// extract from buffer, cast to int
				rightLightVal = (double) (sample[1]);
				synchronized (this) {
					lightData[0] = leftLightVal;
					lightData[1] = rightLightVal;
					// lightData[2] = leftLastLightVal;
					// lightData[3] = rightLastLightVal;
					// lightData[4] = (leftLightVal - leftLastLightVal) / leftLastLightVal;
					// lightData[5] = (rightLightVal - rightLastLightVal) / rightLastLightVal;
				}

				try {
					Thread.sleep(20);
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

	public double[] getValues() {
		double[] result;
		synchronized (this) {
			result = lightData;
		}

		return result;
	}

	public double getLeftValue() {
		double result;
		synchronized (this) {
			result = lightData[0];
		}

		return result;
	}

	public double getRightValue() {
		double result;
		synchronized (this) {
			result = lightData[1];
		}

		return result;
	}

	public void on() {
		this.on = true;

	}

	public void off() {
		this.on = false;
	}

}
