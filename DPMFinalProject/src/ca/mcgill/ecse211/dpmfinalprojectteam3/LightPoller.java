package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class LightPoller extends Thread {
	private EV3ColorSensor sensor;
	private SampleProvider provider;
	private float[] sample;
	private int lightVal = 0;
	private int lastLightVal = 0;

	public LightPoller(EV3ColorSensor sensor, SampleProvider provider) {
		this.sensor = sensor;
		this.provider = provider;

	}

	public void run() {
		sample = new float[1];
		while (true) {
			lastLightVal = lightVal;
			provider.fetchSample(sample, 0); // acquire data
			lightVal = (int) (sample[0]); // extract from buffer, cast to int

			try {
				Thread.sleep(50);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}

	public int getLightVal() {
		return this.lightVal;
	}

	public int getLastLightVal() {
		return this.lastLightVal;
	}
}
