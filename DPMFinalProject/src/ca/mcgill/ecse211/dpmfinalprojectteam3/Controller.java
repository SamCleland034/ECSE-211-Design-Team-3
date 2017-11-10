package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;

/**
 * @since 11/7/17
 * @author Sam
 *
 *         Class that implements the state machine logic we designed for the
 *         project, shown in the software document appendix 4.3. Determines
 *         which method to call based on the current state of the robot. Only
 *         determines state transitions from navigation flag search and
 *         ziptraversal states, the avoidance state is only accessed while in
 *         the navigation state since we need to be navigating in order to
 *         avoid. This class doesn't actually change the state from navigation
 *         to flag search or ziptraversal, it only changes from flag search or
 *         ziptraversal back to navigation since we know explicitly when one of
 *         these algorithms ends. But for navigation, we have to dynamically
 *         figure out when to transition states based on the coordinates that
 *         the robot ends up in after travelling to a position, so we figure
 *         this out through the navigation method startNav().
 * 
 * 
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
	 * @param colorpoller
	 *            the colorpoller
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
	public Controller(Avoidance master, JointLightPoller jointpoller, LightLocalizer lightLoc, LightPoller colorpoller,
			OdometryCorrection oc, SensorRotation sensormotor, UltrasonicLocalizer usLoc, UltrasonicPoller uspoller,
			Navigation gps) {
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
		startingLocalization(uspoller, jointpoller, gps, usLoc, lightLoc);
		waitForLightLOC(lightLoc);
		if (FinalProject.greenTeam == 3) {
			if (FinalProject.greenCorner == 1) {
				FinalProject.odometer.setX(FinalProject.TILE_SPACING);
				FinalProject.odometer.setY(FinalProject.TILE_SPACING);
				FinalProject.odometer.setTheta(0);
			} else if (FinalProject.greenCorner == 2) {
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
			if (FinalProject.redCorner == 1) {
				FinalProject.odometer.setX(FinalProject.TILE_SPACING);
				FinalProject.odometer.setY(FinalProject.TILE_SPACING);
				FinalProject.odometer.setTheta(0);
			} else if (FinalProject.redCorner == 2) {
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
		FinalProject.stage = Stage.NAVIGATION;
		sensormotor.start();
		oc.start();
		// leftpoller.start();
		// rightpoller.start();
		master.start();
		while (true) {
			if (FinalProject.stage == Stage.NAVIGATION) {
				navigating(gps, sensormotor, colorpoller, colorpoller, jointpoller, oc, uspoller, colorpoller);
			} else if (FinalProject.stage == Stage.FLAGSEARCH) {
				System.exit(0);
				flagsearch(gps, master, sensormotor, colorpoller, jointpoller);
				FinalProject.stage = Stage.NAVIGATION;
			} else if (FinalProject.stage == Stage.ZIPTRAVERSAL) {
				ziptraversal(gps, colorpoller, colorpoller, master, lightLoc, sensormotor, colorpoller, jointpoller);
				FinalProject.stage = Stage.NAVIGATION;
			} else if (FinalProject.stage == Stage.FINISHED) {
				Sound.beepSequenceUp();
				System.exit(0);
			}
		}
	}

	private static void waitForLightLOC(LightLocalizer lightLoc) {
		while (lightLoc.localizing)
			continue;
		sleepFor(2);

	}

	/**
	 * Sleep for period of time, created this method because sleep between
	 * transitions of threads very often to makes sure all the actions are smoothly
	 * transitioned between.
	 *
	 * @param i
	 *            the amount we want to sleep for (in seconds)
	 */
	private static void sleepFor(int i) {
		try {
			Thread.sleep(i * 1000);
		} catch (InterruptedException e) {
		}

	}

	/**
	 * Starting localization.
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
		uspoller.on();
		uspoller.start();
		jointpoller.off();
		usLoc.doLocalization();
		uspoller.off();
		jointpoller.on();
		jointpoller.start();
		lightLoc.startLightLOC4();
		waitForLightLOC(lightLoc);

	}

	/**
	 * This method will be called when the robot is in the flagsearch state after
	 * reaching one of the search region corners. We will then turn off the line
	 * detector threads such as odometry correction and left poller and turn on the
	 * ultrasonic related threads except avoidance.
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
		jointpoller.off();
		master.off();
		sensorMotor.off();
		colorpoller.on();
		colorpoller.start();
		if (FinalProject.greenTeam == 3)
			gps.flagSearch(FinalProject.greenColor);
		else
			gps.flagSearch(FinalProject.redColor);

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
	private static void ziptraversal(Navigation gps, LightPoller leftpoller, LightPoller rightpoller, Avoidance master,
			LightLocalizer loc, SensorRotation sensorMotor, LightPoller colorpoller,
			JointLightPoller jointlightpoller) {
		jointlightpoller.on();
		master.off();
		sensorMotor.off();
		colorpoller.off();
		gps.turnTo(0);
		while (Navigation.isNavigating())
			continue;
		sleepFor(2);
		loc.startLightLOC4();
		waitForLightLOC(loc);
		jointlightpoller.off();
		FinalProject.odometer.setX(FinalProject.TILE_SPACING * FinalProject.zipgreenXc);
		FinalProject.odometer.setY(FinalProject.TILE_SPACING * FinalProject.zipgreenYc);
		gps.travelToWithoutAvoid(FinalProject.zipgreenX, FinalProject.zipgreenY);
		while (Navigation.isNavigating())
			continue;
		sleepFor(1);
		gps.zipTraversal();
		FinalProject.odometer.setX(FinalProject.TILE_SPACING * FinalProject.zipredX);
		FinalProject.odometer.setY(FinalProject.TILE_SPACING * FinalProject.zipredY);
		if (FinalProject.odometer.getTheta() <= Math.PI / 4)
			gps.travelToWithoutAvoid(FinalProject.zipredX + 1, FinalProject.zipredY);
		else
			gps.travelToWithoutAvoid(FinalProject.zipredX, FinalProject.zipredY + 1);
		while (Navigation.isNavigating())
			continue;
		sleepFor(2);
		gps.travelToWithoutAvoid(FinalProject.zipredX + 1, FinalProject.zipredY + 1);
		gps.turnTo(0);
		while (Navigation.isNavigating())
			continue;
		jointlightpoller.on();
		loc.startLightLOC4();
		waitForLightLOC(loc);
		FinalProject.odometer.setX(FinalProject.TILE_SPACING * (FinalProject.zipredX + 1));
		FinalProject.odometer.setY(FinalProject.TILE_SPACING * (FinalProject.zipredY + 1));
		FinalProject.odometer.setTheta(0);
		FinalProject.stage = Stage.NAVIGATION;
	}

	/**
	 * This method will be called if the robot is in the navigation phase of the
	 * project. Continuously cycles through the coordinates passed in through the
	 * wifi class. Turns on the threads that will be used during this
	 * FinalProject.stage
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
	public static void navigating(Navigation gps, SensorRotation motor, LightPoller leftPoller, LightPoller rightPoller,
			JointLightPoller jointpoller, OdometryCorrection oc, UltrasonicPoller uspoller, LightPoller colorpoller) {
		colorpoller.off();
		motor.on();
		uspoller.on();
		// leftPoller.on();
		// rightPoller.on();
		jointpoller.on();
		FinalProject.stage = Stage.NAVIGATION;
		gps.startNav();
	}
}
