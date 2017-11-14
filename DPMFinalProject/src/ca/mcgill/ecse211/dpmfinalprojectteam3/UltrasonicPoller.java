package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.robotics.SampleProvider;

// TODO: Auto-generated Javadoc
/**
 * Main thread for polling ultrasonic data, other threads that taking readings
 * from this thread include avoidance and ultrasonic localizer.
 * 
 * @version 1.0
 */
public class UltrasonicPoller extends Thread {

	/** The us. */
	private SampleProvider us;

	/** The us data. */
	private float[] usData;

	/** The master. */
	private Avoidance master;

	/** The gps. */
	private Navigation gps;

	/** The reading. */
	private int reading;

	/** The Constant SAMPLINGPERIOD. */
	private static final int SAMPLINGPERIOD = 25;

	/** The on. */
	private boolean on;

	/**
	 * Instantiates a new ultrasonic poller.
	 *
	 * @param us
	 *            the us
	 * @param usData
	 *            the us data
	 * @param controller
	 *            the controller
	 * @param gps
	 *            the gps
	 */
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
	 * Gets the reading.
	 *
	 * @return the reading
	 */
	public int getReading() {
		return reading;
	}

}
