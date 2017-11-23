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

	/** The Constant 120. Noise margin top for reading distance values */
	// The noise margin is set to be 35 to 55 which is determined experimentally
	private static final int TOP_THRESHOLD = 51; // "d+k"

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

	private float[] usdata = new float[1];

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

		FinalProject.usDist.fetchSample(usdata, 0);
		usdata[0] *= 100;
		if (usdata[0] > TOP_THRESHOLD)
			localizationType = LocalizationType.FALLINGEDGE;
		else
			localizationType = LocalizationType.RISINGEDGE;
		if (localizationType == LocalizationType.RISINGEDGE) { // Starts by facing the wall
			FinalProject.leftMotor.forward(); // Starts turning
			FinalProject.rightMotor.backward();// Turns
			// Get data from ultrasonic sensor
			while (true) {
				FinalProject.usDist.fetchSample(usdata, 0);
				usdata[0] *= 100;
				if (usdata[0] > 70) {
					FinalProject.leftMotor.stop(true);
					FinalProject.rightMotor.stop(false);
					navigation.turnWithoutInterruption(46);
					while (Navigation.isNavigating())
						continue;
					this.localizing = false;
					FinalProject.odometer.setTheta(0);
					return;
				}
			}

		}

		// Not facing the wall
		// The robot should turn until it sees a wall
		else if (localizationType == LocalizationType.FALLINGEDGE) {
			FinalProject.leftMotor.setSpeed(150);
			FinalProject.rightMotor.setSpeed(150);
			FinalProject.leftMotor.forward(); // Starts turning
			FinalProject.rightMotor.backward();

			// Fetching data within the class because of more reliable than via another
			// thread. Since we are focused on the localization we can do it in the main
			// body
			// of code
			FinalProject.usDist.fetchSample(usdata, 0);
			usdata[0] *= 100;
			;

			// Filter out distances that are too close that's not meant to be

			while (usdata[0] >= 50) { // Doesn't see the wall

				FinalProject.usDist.fetchSample(usdata, 0);
				usdata[0] *= 100; // update distance from wall

				if (usdata[0] < 50) {
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
				Thread.sleep(2500);
			} catch (InterruptedException e) {
				// there is nothing to be done here because it is not expected
				// that
				// the odometer will be interrupted by another thread
			}

			; // update distance from wall

			// filter_close(dist); // Filter out distances that are too close for no reasons
			FinalProject.usDist.fetchSample(usdata, 0);
			usdata[0] *= 100;
			// Rotate until it sees a wall
			while (usdata[0] >= 50) { // Doesn't see the wall

				FinalProject.usDist.fetchSample(usdata, 0);
				usdata[0] *= 100; // update distance from wall

				if (usdata[0] < 50) {
					SECOND_ANGLE = odometer.getTheta();
					FinalProject.leftMotor.stop(true);
					FinalProject.rightMotor.stop(false); // Sees the wall
					Sound.buzz();
					Sound.playTone(1200, 100);

				}
			}

			// Save the angle read by the odometer
			updateAngle(FIRST_ANGLE, SECOND_ANGLE); // turn to face (0 axis)

		}
		this.localizing = false;
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
			UPDATED_ANGLE = 45 - ((first_angle + second_angle) / 2.0);
			navigation.turn(UPDATED_ANGLE);
		}

		if (first_angle > second_angle) {
			UPDATED_ANGLE = 225 - ((first_angle + second_angle) / 2.0);
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

}