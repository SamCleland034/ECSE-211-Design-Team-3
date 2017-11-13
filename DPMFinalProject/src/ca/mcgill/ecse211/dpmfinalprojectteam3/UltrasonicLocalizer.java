
package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;

/**
 * This class is used in the start to approximate a suitable heading to then
 * perform light localization using the light sensor.
 * 
 * 
 */
public class UltrasonicLocalizer {

	/**
	 * The Enum LocalizationType.
	 */
	// Define the two types of correction

	/** The rotate speed. Rotating speed for ultrasonic localization */
	// Define variables
	public static float ROTATE_SPEED = 70;

	/** The dist. */
	private double dist;
	private UltrasonicPoller poller;
	/**
	 * The updated angle. Angle we have to turn to once we calculate the new heading
	 */
	private double UPDATED_ANGLE;

	/** The Constant TOP_THRESHOLD. Noise margin top for reading distance values */
	// The noise margin is set to be 35 to 55 which is determined experimentally
	private static final int TOP_THRESHOLD = 51; // "d+k"

	/**
	 * The Constant BOTTOM_THRESHOLD. Noise margin bottom for reading distance
	 * values
	 */
	private static final int BOTTOM_THRESHOLD = 35; // "d-k"

	/** The Constant FILTER_OUT. To filter out bad data */
	// Used to filter values
	private static final int FILTER_OUT = 30;

	private static final int FILTER_CONTROL = 130;

	/** The odometer. */
	private Odometer odometer;

	/** The navigation. */
	private Navigation navigation;

	/** The localization type. */
	private ca.mcgill.ecse211.dpmfinalprojectteam3.LocalizationType localizationType;

	/** The filter control. */
	private int filterControl;

	public boolean localizing;

	private int filter = 0;

	/**
	 * Instantiates a new ultrasonic localizer.
	 *
	 * @param leftMotor
	 *            the left motor
	 * @param rightMotor
	 *            the right motor
	 * @param odometer
	 *            the odometer, keep track of the wheel rotations to get an angle to
	 *            turn to
	 * @param navigation
	 *            the navigation, using turn to method to determine where we need to
	 *            turn to after calculating theta values
	 * @param lt
	 *            the localization type, determine if rising or falling edge
	 */
	public UltrasonicLocalizer(Odometer odometer, Navigation navigation,
			ca.mcgill.ecse211.dpmfinalprojectteam3.LocalizationType lt, UltrasonicPoller poller) {
		this.odometer = odometer;
		this.localizationType = lt;
		this.navigation = navigation;
		this.poller = poller;
		this.localizing = true;

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	/*
	 * public void run() {
	 * 
	 * try { Thread.sleep(2000); } catch (InterruptedException e) { // there is
	 * nothing to be done here because it is not expected that // the odometer will
	 * be interrupted by another thread }
	 * 
	 * doLocalize(); // Start localizing }
	 */

	/**
	 * Do localize, main ultrasonic localization method that will perform the
	 * ultrasonic localization seen in lab 4
	 */
	public void doLocalization() {

		double FIRST_ANGLE = 0, SECOND_ANGLE = 0; // To store two angles when it changes from seeing the wall to facing
		// away or vice versa
		this.localizing = true;
		FinalProject.leftMotor.setSpeed(150);
		FinalProject.rightMotor.setSpeed(150);
		if (localizationType == LocalizationType.RISINGEDGE) { // Starts by facing the wall
			FinalProject.leftMotor.forward(); // Starts turning
			FinalProject.rightMotor.backward();// Turns
			this.dist = 0;
			// Get data from ultrasonic sensor
			while (true) {
				this.dist = processUSData();
				if (this.dist > TOP_THRESHOLD) {
					FinalProject.leftMotor.stop(true);
					FinalProject.rightMotor.stop(false);
					navigation.turnWithoutInterruption(40);
					while (Navigation.isNavigating())
						continue;
					this.localizing = false;
					FinalProject.odometer.setTheta(0);
					return;
				}
			} // Filter the distances that is too far that's not meant to be
			/*
			 * while (this.dist < BOTTOM_THRESHOLD) { // It's seeing the wall during the
			 * turn
			 * 
			 * this.dist = poller.getReading(); // update distance from wall
			 * 
			 * if (this.dist > BOTTOM_THRESHOLD) { FIRST_ANGLE = odometer.getTheta(); //
			 * Stop the motos when it doesn't see the wall anymore Sound.buzz();
			 * FinalProject.leftMotor.stop(true); FinalProject.rightMotor.stop(false); } }
			 */
			// Save the angle read by the odometer

		}

		// Not facing the wall
		// The robot should turn until it sees a wall
		else if (localizationType == LocalizationType.FALLINGEDGE) {
			FinalProject.leftMotor.setSpeed(150);
			FinalProject.rightMotor.setSpeed(150);
			FinalProject.leftMotor.forward(); // Starts turning
			FinalProject.rightMotor.backward();
			// Fetch data

			this.dist = poller.getReading();
			;

			filter_close(dist); // Filter out distances that are too close that's not meant to be

			while (this.dist >= TOP_THRESHOLD) { // Doesn't see the wall

				this.dist = poller.getReading(); // update distance from wall

				if (this.dist < TOP_THRESHOLD) {
					FIRST_ANGLE = odometer.getTheta();
					FinalProject.leftMotor.stop(true);
					FinalProject.rightMotor.stop(false);
					// When it sees the wall, stop
					Sound.buzz();

				}

			}

			// Save the angle read by the odometer

			// Rotate until the robot sees no wall
			Sound.buzz();
			FinalProject.leftMotor.backward();
			FinalProject.rightMotor.forward();
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// there is nothing to be done here because it is not expected
				// that
				// the odometer will be interrupted by another thread
			}
			this.dist = poller.getReading();
			; // update distance from wall

			filter_close(dist); // Filter out distances that are too close for no reasons

			// Rotate until it sees a wall
			while (this.dist >= TOP_THRESHOLD) { // Doesn't see the wall

				this.dist = poller.getReading(); // update distance from wall

				if (this.dist < TOP_THRESHOLD) {
					SECOND_ANGLE = odometer.getTheta();
					FinalProject.leftMotor.stop(true);
					FinalProject.rightMotor.stop(false); // Sees the wall
					Sound.buzz();
					Sound.playTone(1000, 100);

				}
			}

			// Save the angle read by the odometer
			updateAngle(FIRST_ANGLE, SECOND_ANGLE); // turn to face (0 axis)

		}
		this.localizing = false;
	}

	private double processUSData() {
		double data = poller.getReading();
		if (filter < FILTER_CONTROL && data > 51) {
			filter++;
			return this.dist;
		} else if (filter >= FILTER_CONTROL && data > 51) {
			return data;
		} else {
			filter = 0;
			return data;
		}

	}

	/**
	 * Update angle. Update angle to the correct heading that we need to adjust to
	 *
	 * @param first_angle
	 *            the first angle recorded
	 * @param second_angle
	 *            the second angle recorded
	 */
	public void updateAngle(double first_angle, double second_angle) {

		// Computes the correct angle to turn using the the angles collected
		if (first_angle < second_angle) {
			UPDATED_ANGLE = 45 - ((first_angle + second_angle) / 2);
			navigation.turn(UPDATED_ANGLE);
		}

		if (first_angle > second_angle) {
			UPDATED_ANGLE = 225 - ((first_angle + second_angle) / 2);
			navigation.turn(UPDATED_ANGLE);
		}

		if (localizationType == LocalizationType.FALLINGEDGE) {
			UPDATED_ANGLE = UPDATED_ANGLE + 17; // This value is found experimentally
			navigation.turnWithoutInterruption(UPDATED_ANGLE);
		}

		else if (localizationType == LocalizationType.RISINGEDGE) {
			UPDATED_ANGLE = UPDATED_ANGLE + 200; // This value is found experimentally
			navigation.turnWithoutInterruption(UPDATED_ANGLE);
		}
	}

	/**
	 * Filter far, filter out far distances unless they get repeated enough to
	 * suggest nothing is ther
	 *
	 * @param dist
	 *            the dist, distance the ultrasonic sensor reads
	 * @return the double, the distance returned based on filtering or no filtering
	 */
	// Filter the distances that is too far that's not meant to be
	private double filter_far(double dist) {

		if (dist >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (dist >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.dist = dist;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.dist = dist;
		}
		return this.dist;
	}

	/**
	 * Filter close, filter out close data unless it is repeated a lot , which means
	 * we are actually close to something
	 *
	 * @param dist
	 *            the dist, distance ultrasonic sensor reads
	 * @return the double, distance value returned
	 */
	// Filter out distances that are too close for no reasons
	private double filter_close(double dist) {

		if (dist <= 20 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (dist <= 20) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.dist = dist;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.dist = dist;
		}
		return this.dist;
	}

}
