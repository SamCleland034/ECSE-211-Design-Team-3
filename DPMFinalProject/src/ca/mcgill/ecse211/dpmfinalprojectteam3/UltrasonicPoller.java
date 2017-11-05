package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.robotics.SampleProvider;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller
 * thread. The while loop at the bottom executes in a loop. Assuming that the
 * us.fetchSample, and cont.processUSData methods operate in about 20mS, and
 * that the thread sleeps for 50 mS at the end of each loop, then one cycle
 * through the loop is approximately 70 mS. This corresponds to a sampling rate
 * of 1/70mS or about 14 Hz.
 * 
 * @version 1.0
 */
public class UltrasonicPoller extends Thread {
	private SampleProvider us;
	private float[] usData;
	private Avoidance master;
	private Navigation gps;
	private int reading;
	private static final int SAMPLINGPERIOD = 45;
	private boolean on;

	public UltrasonicPoller(SampleProvider us, float[] usData, Avoidance controller, Navigation gps) {
		this.us = us;
		this.usData = usData;
		this.master = controller;
		controller.setPoller(this);
		this.gps = gps;
		gps.setPoller(this);
		this.on = false;

	}

	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result
	 * to an integer [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		long startTime, endTime;
		while (true) {
			if (on) {
				startTime = System.currentTimeMillis();
				us.fetchSample(usData, 0); // acquire data
				reading = (int) (usData[0] * 100.0); // extract from buffer, cast to int
				if (reading > 255)
					reading = 255;
				endTime = System.currentTimeMillis();
				if (endTime - startTime < SAMPLINGPERIOD) {
					try {
						Thread.sleep(SAMPLINGPERIOD - (endTime - startTime));
					} catch (Exception e) {
					}
				} // Poor man's timed sampling
			} else {
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
				}
			}

		}
	}

	public void on() {
		on = true;
	}

	public void off() {
		on = false;
	}

	public int getReading() {
		return reading;
	}

}
