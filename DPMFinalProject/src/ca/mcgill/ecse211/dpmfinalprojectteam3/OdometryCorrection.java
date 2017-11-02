
package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * The Class OdometryCorrection, used to correct the small but accumulating
 * errors of the odometer using the light sensor to detect gridlines on the
 * floor
 * 
 * @version 1.0
 */
public class OdometryCorrection extends Thread {

	/** The Constant CORRECTION_PERIOD. Sampling rate of light sensor data */
	private static final long CORRECTION_PERIOD = 10;

	/** The number of turns robot has made. */
	private static int nbrOfTurns = 0;

	/** The number of lines crossed by the robot. */
	private static int nbrOfLines = 0;

	/** The x. */
	private static double x;

	/** The y. */
	private static double y;

	/** The x offset. */
	private static double xOffset;

	/** The y offset. */
	private static double yOffset;

	/** The odometer. */
	private Odometer odometer;

	/** The distance between lines. */
	private static double distanceBetweenLines = 30.48;

	// private EV3ColorSensor colorSensor;

	/** The Constant LightPort. */
	private static final Port LightPort = LocalEV3.get().getPort("S1"); // Assign a port

	// constructor

	/**
	 * Instantiates a new odometry correction.
	 *
	 * @param odometer
	 *            the odometer
	 */
	public OdometryCorrection(Odometer odometer) {

		this.odometer = odometer;

	}

	// run method (required for Thread)

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {

		long correctionStart, correctionEnd;

		@SuppressWarnings("resource")

		EV3ColorSensor colorSensor = new EV3ColorSensor(LightPort);

		colorSensor.setFloodlight(lejos.robotics.Color.WHITE);

		while (true) {

			correctionStart = System.currentTimeMillis();

			// TODO Place correction implementation here

			colorSensor.getColorIDMode();

			SampleProvider provider = colorSensor.getMode("ColorID");

			float colorSamples[] = new float[100];

			colorSensor.fetchSample(colorSamples, 1);

			int color = (int) colorSamples[1];

			if (Navigation.isNavigating()) { // turning

				nbrOfTurns++; // Increment the number of turns at every turn

				// To make sure it only counts one turn at every turn

				try {

					Thread.sleep(2500);

				} catch (InterruptedException e) {

					// TODO Auto-generated catch block

					e.printStackTrace();

				}

				nbrOfLines = 0; // reset count of lines after each turn

			} else { // not turning, moving

				if (color == 13) { // If it sees a black line, beep

					Sound.beepSequenceUp();

					if (nbrOfTurns == 0) {

						if (nbrOfLines == 0) { // At the first line in front of the first square

							y = 0;

							odometer.setY(y);

						} else { // At the lines after the first line when the robot has not turned yet

							y = y + distanceBetweenLines; // Increment y by the distance between the lines at every line
															// detected

							odometer.setY(y);

						}

					}

					if (nbrOfTurns == 1) { // When it reaches its first line after its first turn

						if (nbrOfLines == 0) {

							x = 0;

							odometer.setX(x);

						} else { // At the lines after the first line after its first turn

							x = x + distanceBetweenLines; // Increment x by the distance between the lines at every line
															// detected

							odometer.setX(x);

						}

					}

					if (nbrOfTurns == 2) { // When it reaches its first line after its second turn

						if (nbrOfLines == 0) {

							odometer.setY(y);

						} else { // At the lines after the first line after its second turn

							y = y - distanceBetweenLines; // Decrement y by the distance between the lines at every line
															// detected

							odometer.setY(y);

						}

					}

					if (nbrOfTurns == 3) { // When it reaches its first line after its third turn

						if (nbrOfLines == 0) {

							odometer.setX(x);

						} else { // At the lines after the first line after its third turn

							x = x - distanceBetweenLines; // Decrement x by the distance between the lines at every line
															// detected

							odometer.setX(x);

						}

					}

					nbrOfLines++;

					// Stop the robot after each turn to help the sensor to detect the lines after a
					// turn

					try {

						Thread.sleep(1000);

					} catch (InterruptedException e) {
					}

				}

			}

			// this ensure the odometry correction occurs only once every period

			correctionEnd = System.currentTimeMillis();

			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {

				try {

					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));

				} catch (InterruptedException e) {

					// there is nothing to be done here because it is not

					// expected that the odometry correction will be

					// interrupted by another thread

				}

			}

		}

	}

}