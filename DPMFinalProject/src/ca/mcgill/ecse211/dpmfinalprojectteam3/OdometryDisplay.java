/*
 * OdometryDisplay.java
 */

package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.lcd.TextLCD;

/**
 * Uses to display the odometer to the screen of the EV3 brick. 
 * Also displays other things such as the ultrasonic sensor readings
 * 
 * @version 1.0
 */
public class OdometryDisplay extends Thread {

	/** The Constant DISPLAY_PERIOD. */
	private static final long DISPLAY_PERIOD = 100;

	/** The odometer. */
	private Odometer odometer;

	/** The t. */
	private TextLCD t;

	/**
	 * Instantiates a new odometry display.
	 *
	 * @param odometer
	 *            the odometer that will be displayed on the screen
	 * @param t
	 *            the t, the actual display
	 */
	// constructor
	public OdometryDisplay(Odometer odometer, TextLCD t) {
		this.odometer = odometer;
		this.t = t;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	// run method (required for Thread)
	public void run() {
		long displayStart, displayEnd;
		double[] position = new double[3];

		// clear the display once
		t.clear();

		while (true) {
			displayStart = System.currentTimeMillis();

			// clear the lines for displaying odometry information
			t.drawString("X:              ", 0, 0);
			t.drawString("Y:              ", 0, 1);
			t.drawString("T:              ", 0, 2);
			t.drawString("Distance:       ", 0, 3);

			// get the odometry information
			odometer.getPosition(position, new boolean[] { true, true, true });

			// display odometry information
			for (int i = 0; i < 3; i++) {
				t.drawString(formattedDoubleToString(position[i], 2), 3, i);
			}

			// get the ultrasonic sensor information
			FinalProject.usSensor.fetchSample(FinalProject.sample, 0);

			// display the ultrasonic sensor information
			t.drawString(Float.toString(FinalProject.sample[0] * 100), 3, 3);

			// throttle the OdometryDisplay
			displayEnd = System.currentTimeMillis();
			if (displayEnd - displayStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that OdometryDisplay will be interrupted
					// by another thread
				}
			}
		}
	}

	/**
	 * Formatted double to string.
	 *
	 * @param x
	 *            the x, converts input to string
	 * @param places
	 *            the places, used to put decimal if needed
	 * @return the string 
	 * 			  the string, which is the result from the initial string input
	 */
	private static String formattedDoubleToString(double x, int places) {
		String result = "";
		String stack = "";
		long t;

		// put in a minus sign as needed
		if (x < 0.0)
			result += "-";

		// put in a leading 0
		if (-1.0 < x && x < 1.0)
			result += "0";
		else {
			t = (long) x;
			if (t < 0)
				t = -t;

			while (t > 0) {
				stack = Long.toString(t % 10) + stack;
				t /= 10;
			}

			result += stack;
		}

		// put the decimal, if needed
		if (places > 0) {
			result += ".";

			// put the appropriate number of decimals
			for (int i = 0; i < places; i++) {
				x = Math.abs(x);
				x = x - Math.floor(x);
				x *= 10.0;
				result += Long.toString((long) x);
			}
		}

		return result;
	}

}
