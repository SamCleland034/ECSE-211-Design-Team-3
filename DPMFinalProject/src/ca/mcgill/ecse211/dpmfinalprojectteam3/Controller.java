package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;

/**
 * Class that implements the state machine logic we designed for the project,
 * shown in the software document, final software design (5.0). Determines which
 * method to call based on the current state of the robot. Only determines state
 * transitions from navigation, flag search and ziptraversal states. The
 * avoidance state is only accessed while in the navigation state since we need
 * to be navigating in order to avoid. This class doesn't actually change the
 * state from navigation to flag search or ziptraversal, it only changes from
 * flag search or ziptraversal back to navigation since we know explicitly when
 * one of these algorithms ends. But for navigation, we have to figure out when
 * to transition states based on the coordinates that the robot ends up in after
 * travelling to a position, so we figure this out through the navigation method
 * startNav().
 * 
 * @since 11/7/17
 * @author Sam
 *
 */
public class Controller {

	/** The master. */
	private Avoidance master;

	/** The jointpoller. */
	private JointLightPoller jointpoller;

	/** The light loc. */
	private LightLocalizer lightLoc;

	/** The oc. */
	private OdometryCorrection oc;

	/** The sensormotor. */
	private SensorRotation sensormotor;

	/** The us loc. */
	private UltrasonicLocalizer usLoc;

	/** The uspoller. */
	private UltrasonicPoller uspoller;

	/** The gps. */
	private Navigation gps;

	/** The colorpoller. */
	private LightPoller colorpoller;

	/**
	 * Instantiates a new controller.
	 *
	 * @param master
	 *            the master
	 * @param jointpoller
	 *            the jointpoller
	 * @param lightLoc
	 *            the light loc
	 * @param oc
	 *            the oc
	 * @param sensormotor
	 *            the sensormotor
	 * @param usLoc
	 *            the us loc
	 * @param uspoller
	 *            the uspoller
	 * @param gps
	 *            the gps
	 */
	public Controller(Avoidance master, JointLightPoller jointpoller, LightLocalizer lightLoc, OdometryCorrection oc,
			SensorRotation sensormotor, UltrasonicLocalizer usLoc, UltrasonicPoller uspoller, Navigation gps,
			LightPoller colorpoller) {
		this.master = master;
		this.jointpoller = jointpoller;
		this.lightLoc = lightLoc;
		this.colorpoller = colorpoller;
		this.oc = oc;
		this.sensormotor = sensormotor;
		this.usLoc = usLoc;
		this.uspoller = uspoller;
		this.gps = gps;
	}

	/**
	 * Starts the state machine logic that we display in our state machine model of
	 * the capture the flag game. Will start from the startinglocalization state
	 * initally.
	 * 
	 */
	public void startControlFlow() {
		FinalProject.stage = Stage.STARTINGLOCALIZATION;
		// localize in corner
		startingLocalization(uspoller, jointpoller, gps, usLoc, lightLoc);
		waitForLightLOC(lightLoc);
		// set coordinates to these based on what color and corner starting from
		if (FinalProject.greenTeam == 3) {
			if (FinalProject.greenCorner == 0) {
				FinalProject.odometer.setX(FinalProject.TILE_SPACING);
				FinalProject.odometer.setY(FinalProject.TILE_SPACING);
				FinalProject.odometer.setTheta(0);
			} else if (FinalProject.greenCorner == 1) {
				FinalProject.odometer.setX(7 * FinalProject.TILE_SPACING);
				FinalProject.odometer.setY(FinalProject.TILE_SPACING);
				FinalProject.odometer.setTheta(3 * Math.PI / 2);
			} else if (FinalProject.greenCorner == 2) {
				FinalProject.odometer.setX(7 * FinalProject.TILE_SPACING);
				FinalProject.odometer.setY(7 * FinalProject.TILE_SPACING);
				FinalProject.odometer.setTheta(Math.PI);
			} else {
				FinalProject.odometer.setX(FinalProject.TILE_SPACING);
				FinalProject.odometer.setY(7 * FinalProject.TILE_SPACING);
				FinalProject.odometer.setTheta(Math.PI / 2);
			}
		} else {
			if (FinalProject.redCorner == 0) {
				FinalProject.odometer.setX(FinalProject.TILE_SPACING);
				FinalProject.odometer.setY(FinalProject.TILE_SPACING);
				FinalProject.odometer.setTheta(0);
			} else if (FinalProject.redCorner == 1) {
				FinalProject.odometer.setX(7 * FinalProject.TILE_SPACING);
				FinalProject.odometer.setY(FinalProject.TILE_SPACING);
				FinalProject.odometer.setTheta(3 * Math.PI / 2);
			} else if (FinalProject.redCorner == 2) {
				FinalProject.odometer.setX(7 * FinalProject.TILE_SPACING);
				FinalProject.odometer.setY(7 * FinalProject.TILE_SPACING);
				FinalProject.odometer.setTheta(Math.PI);
			} else {
				FinalProject.odometer.setX(FinalProject.TILE_SPACING);
				FinalProject.odometer.setY(7 * FinalProject.TILE_SPACING);
				FinalProject.odometer.setTheta(Math.PI / 2);
			}
		}
		// change state
		FinalProject.stage = Stage.NAVIGATION;
		// start threads
		// sensormotor.start();
		oc.start();
		// leftpoller.start();
		// rightpoller.start();
		// master.start();
		while (true) {

			if (FinalProject.stage == Stage.NAVIGATION) {
				// start navigating
				navigating(gps, sensormotor, jointpoller, oc, uspoller, colorpoller);
			} else if (FinalProject.stage == Stage.FLAGSEARCH) {
				// start flagsearch
				flagsearch(gps, master, sensormotor, colorpoller, jointpoller);
				FinalProject.stage = Stage.NAVIGATION;
			} else if (FinalProject.stage == Stage.ZIPTRAVERSAL) {
				// start ziptraversal
				ziptraversal(gps, master, lightLoc, sensormotor, jointpoller);
				FinalProject.stage = Stage.NAVIGATION;
			} else if (FinalProject.stage == Stage.FINISHED) {
				travelToCorner(gps, master, sensormotor, jointpoller, oc);
				Sound.beepSequenceUp();
				System.exit(0);
			}
		}
	}

	private void travelToCorner(Navigation gps, Avoidance master, SensorRotation sensormotor,
			JointLightPoller jointpoller, OdometryCorrection oc) {
		oc.off();
		master.off();
		sensormotor.off();
		jointpoller.off();

		if (FinalProject.greenTeam == 3) {
			switch (FinalProject.greenCorner) {
			case 0:
				gps.travelToWithoutAvoid(0.5, 0.5);
				break;
			case 1:
				gps.travelToWithoutAvoid(7.5, 0.5);
				break;
			case 2:
				gps.travelToWithoutAvoid(7.5, 7.5);
				break;
			case 3:
				gps.travelToWithoutAvoid(0.5, 7.5);
				break;
			}
		} else {
			switch (FinalProject.redCorner) {
			case 0:
				gps.travelToWithoutAvoid(0.5, 0.5);
				break;
			case 1:
				gps.travelToWithoutAvoid(7.5, 0.5);
				break;
			case 2:
				gps.travelToWithoutAvoid(7.5, 7.5);
				break;
			case 3:
				gps.travelToWithoutAvoid(0.5, 7.5);
				break;
			}
		}
		while (Navigation.isNavigating())
			continue;
	}

	private static void waitForLightLOC(LightLocalizer lightLoc) {
		while (lightLoc.localizing)
			continue;
		sleepFor(0.2);

	}

	/**
	 * Sleep for period of time, created this method because sleep between
	 * transitions of threads very often to makes sure all the actions are smoothly
	 * transitioned between.
	 *
	 * @param i
	 *            the amount we want to sleep for (in seconds)
	 */
	private static void sleepFor(double i) {
		try {
			double time = i * 1000;
			Thread.sleep((long) time);
		} catch (InterruptedException e) {
		}

	}

	/**
	 * Starting localization, first thing that the robot does corresponding to the
	 * game logic
	 *
	 * @param uspoller
	 *            the uspoller
	 * @param jointpoller
	 *            the jointpoller
	 * @param gps
	 *            the gps
	 * @param usLoc
	 *            the us loc
	 * @param lightLoc
	 *            the light loc
	 */
	private static void startingLocalization(UltrasonicPoller uspoller, JointLightPoller jointpoller, Navigation gps,
			UltrasonicLocalizer usLoc, LightLocalizer lightLoc) {
		// turn on uspoller
		// uspoller.on();
		uspoller.off();
		uspoller.start();
		sleepFor(0.1);
		// don't need light pollers for US localization
		jointpoller.off();
		// start us loc
		usLoc.doLocalization();
		while (usLoc.localizing)
			continue;
		// don't need us poller for light localization
		uspoller.off();
		// turn on light poller
		jointpoller.on();

		jointpoller.start();
		sleepFor(0.1);
		// start light localization
		lightLoc.sweepRight();
		lightLoc.startLightLOC4();
		waitForLightLOC(lightLoc);

	}

	/**
	 * This method will be called when the robot is in the flagsearch state after
	 * reaching one of the search region corners. We will then turn off the line
	 * detector threads such as odometry correction and jointlightpoller and turn on
	 * the ultrasonic related threads except avoidance.
	 *
	 * @param gps
	 *            the gps
	 * @param master
	 *            the master
	 * @param sensorMotor
	 *            the sensor motor
	 * @param colorpoller
	 *            the colorpoller
	 * @param jointpoller
	 *            the jointpoller
	 */
	public static void flagsearch(Navigation gps, Avoidance master, SensorRotation sensorMotor, LightPoller colorpoller,
			JointLightPoller jointpoller) {
		// leftpoller.off();
		// rightpoller.off();
		// don't need light poller for OC
		jointpoller.off();
		// dont need avoidance
		master.off();
		// dont need US motor until we sweep
		sensorMotor.off();
		// need to start flagsearch light poller
		colorpoller.on();
		colorpoller.start();
		gps.flagSearch(FinalProject.RGBColors);

	}

	/**
	 * This robot will be called if the robot switches into the ziptraversal
	 * FinalProject.stage, which will then turn off all the threads that we don't
	 * want on for the zip traversal such as odometry correction, avoidance and
	 * ultrasonic poller.
	 *
	 * @param gps
	 *            the gps
	 * @param leftpoller
	 *            the leftpoller
	 * @param rightpoller
	 *            the rightpoller
	 * @param master
	 *            the master
	 * @param loc
	 *            the loc
	 * @param sensorMotor
	 *            the sensor motor
	 * @param colorpoller
	 *            the colorpoller
	 * @param jointlightpoller
	 *            the jointlightpoller
	 */
	private static void ziptraversal(Navigation gps, Avoidance master, LightLocalizer loc, SensorRotation sensorMotor,
			JointLightPoller jointlightpoller) {
		// need lightpoller for light loc
		jointlightpoller.on();
		// don't need avoidance
		master.off();
		// don't need us motor
		sensorMotor.off();
		// don't need colorpoller incase it is on

		// need to turn to 0 for correct localization
		gps.turnTo(0);
		while (Navigation.isNavigating())
			continue;
		sleepFor(0.6);
		// loc.correctPosition();
		// start localization at the zipline orientation coordinates
		loc.sweepLeft();
		loc.startLightLOC4();
		waitForLightLOC(loc);
		// experimental turn
		// gps.turn(4);
		while (Navigation.isNavigating())
			continue;
		FinalProject.odometer.setTheta(0);
		// set odometer to 0 since localize finished
		jointlightpoller.off();
		// set the coordinates of wherever we localized at, in this case it would be
		// zipgreen coords
		FinalProject.odometer.setX(FinalProject.TILE_SPACING * FinalProject.zipgreenXc);
		FinalProject.odometer.setY(FinalProject.TILE_SPACING * FinalProject.zipgreenYc);
		// travel to the zipline to get ready for the zip traversal
		gps.travelToWithoutAvoid(FinalProject.zipgreenX, FinalProject.zipgreenY);

		sleepFor(0.4);
		// get inital theta value for when we are traversing the zipline
		double initalTheta = gps.zipTraversal();
		while (Navigation.isNavigating())
			continue;
		while (gps.ziptraversing)
			continue;
		// after traversing, set the odometer coords to the zip red endpoints
		FinalProject.odometer.setX(FinalProject.TILE_SPACING * FinalProject.zipredX);
		FinalProject.odometer.setY(FinalProject.TILE_SPACING * FinalProject.zipredY);
		FinalProject.odometer.setTheta(initalTheta);
		// approximately travel to the red orientation points to localize
		gps.travelToWithoutAvoid(FinalProject.zipredXc, FinalProject.zipredYc);
		// checkOrientation(initalTheta, gps);

		while (Navigation.isNavigating())
			continue;
		sleepFor(0.3);
		// gps.travelToWithoutAvoid(FinalProject.zipredX + 1, FinalProject.zipredY + 1);
		// turn to 0 for light localization
		gps.turnTo(0);
		jointlightpoller.on();
		// perform a sweep just incase the robot is on a line, if it is on a line, it
		// will reposition to the left to reposition correctly
		loc.sweepLeft();
		while (Navigation.isNavigating())
			continue;

		sleepFor(0.2);
		// relocalization on other end of zipline
		loc.startLightLOC4();
		waitForLightLOC(loc);
		// set the coordinates to wherever we just localized at, in this case it is
		// zipredc coords
		FinalProject.odometer.setX(FinalProject.zipredXc * FinalProject.TILE_SPACING);
		FinalProject.odometer.setY(FinalProject.zipredYc * FinalProject.TILE_SPACING);
		FinalProject.odometer.setTheta(0);
		FinalProject.stage = Stage.NAVIGATION;
	}

	/**
	 * This method will be called if the robot is in the navigation phase of the
	 * project. Continuously cycles through the coordinates passed in through the
	 * wifi class. Turns on the threads that will be used during this stage such as
	 * odometry correction, uspoller, jointlightpoller, avoidance and sensormotor.
	 *
	 * @param gps
	 *            the gps
	 * @param motor
	 *            the motor
	 * @param leftPoller
	 *            the left poller
	 * @param rightPoller
	 *            the right poller
	 * @param jointpoller
	 *            the jointpoller
	 * @param oc
	 *            the oc
	 * @param uspoller
	 *            the uspoller
	 * @param colorpoller
	 *            the colorpoller
	 */
	public static void navigating(Navigation gps, SensorRotation motor, JointLightPoller jointpoller,
			OdometryCorrection oc, UltrasonicPoller uspoller, LightPoller colorpoller) {
		// turn on threads that need to be on for navigation
		colorpoller.off();
		motor.on();
		uspoller.on();
		// leftPoller.on();
		// rightPoller.on();
		jointpoller.on();
		gps.startNav();
	}
}
