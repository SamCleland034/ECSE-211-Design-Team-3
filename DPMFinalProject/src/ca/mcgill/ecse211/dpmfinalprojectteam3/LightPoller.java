package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class LightPoller extends Thread {
	private EV3ColorSensor sensor;
	private SampleProvider provider;
	private float[] sample;
	private double lightVal = 0;
	private double lastLightVal = 0;
	public boolean on;

	public LightPoller(EV3ColorSensor sensor, SampleProvider provider) {
		this.sensor = sensor;
		this.provider = provider;
		on = false;

	}

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

	public double getChangeInLight() {
		double result;
		synchronized (this) {
			result = (lightVal - lastLightVal) / lastLightVal;
		}
		return result;
	}

	public double getLightVal() {
		double result;
		synchronized (this) {
			result = lightVal;
		}

		return result;
	}

	public double getLastLightVal() {
		double result;
		synchronized (this) {
			result = lastLightVal;
		}
		return result;
	}

	public void on() {
		on = true;
	}

	public void off() {
		on = false;
	}
}
