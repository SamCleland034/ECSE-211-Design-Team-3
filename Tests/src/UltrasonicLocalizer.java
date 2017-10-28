

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

// TODO: Auto-generated Javadoc
/**
 * The Class UltrasonicLocalizer, used in the start to approximate a suitable
 * heading to then perform light localization using the light sensor.
 */
public class UltrasonicLocalizer extends Thread {

	/**
	 * The Enum LocalizationType.
	 */
	// Define the two types of correction
	public enum LocalizationType {

		/** The falling edge. */
		FALLING_EDGE,
		/** The rising edge. */
		RISING_EDGE
	};

	/** The rotate speed. Rotating speed for ultrasonic localization */
	// Define variables
	public static float ROTATE_SPEED = 70;
	ArrayList<Double> usdata= new ArrayList<Double>();
	/** The distance. Distance that the ultrasonic sensor reads */
	private double distance;

	/** The dist. */
	private double dist;

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

	/** The Constant MAX_DISTANCE_BETWEEN. */
	private static final int MAX_DISTANCE_BETWEEN = 43; // "d"

	/** The Constant FILTER_OUT. To filter out bad data */
	// Used to filter values
	private static final int FILTER_OUT = 20;

	/** The odometer. */
	private Odometer odometer;

	/** The navigation. */
	private Navigation navigation;

	/** The localization type. */
	private LocalizationType localizationType;

	/** The filter control. */
	private int filterControl;

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
	 * @param localizationType
	 *            the localization type, determine if rising or falling edge
	 */
	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			Navigation navigation, LocalizationType localizationType) {
		this.odometer = odometer;
		this.localizationType = localizationType;
		this.navigation = navigation;

		// Reset the motors
		for (EV3LargeRegulatedMotor motors : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motors.stop();
			motors.setSpeed(120);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {

		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}

		doLocalize(); // Start localizing
	}

	/**
	 * Do localize, main ultrasonic localization method that will perform the actual
	 * functionality we need to do
	 */
	public void doLocalize() {
		
		double FIRST_ANGLE = 0, SECOND_ANGLE = 0; // To store two angles when it changes from seeing the wall to facing
		// away or vice versa

		if (localizationType == LocalizationType.RISING_EDGE) { // Starts by facing the wall
			//navigation.turn(360); // Turns
			Test.leftMotor.forward();
			Test.rightMotor.backward();
			long startTime=0;
			long endTime=0;
			int counter =0;
			long period= 50;
			while (counter < 320) {
			startTime= System.currentTimeMillis();
			Test.usSensor.fetchSample(Test.sample, 0); // Get data from ultrasonic sensor
			double dist = Test.sample[0] * 100;
			if(dist>255) dist= 255;
			System.out.println(dist);
			usdata.add(dist);
			counter++;
			endTime = System.currentTimeMillis();
			if(endTime-startTime<50){
				try {
					Thread.sleep(50-(endTime-startTime));
				} catch (InterruptedException e) {}
			}
			}
			Test.leftMotor.stop(true);
			Test.rightMotor.stop(false);
			double sum=0;
			double mean=0;
			double stDev=0;
			for(int i =0; i<usdata.size();i++){
				sum+= usdata.get(i);
			}
			mean = sum/usdata.size();
			sum= 0;
			for(int i=0; i<usdata.size();i++){
				sum+=Math.pow(usdata.get(i)-mean,2);
			}
			stDev= Math.sqrt(sum/usdata.size()-1);
			System.out.println("Mean = " + mean) ;
			System.out.println("Standard Dev = " + stDev);
			
//			filter_far(dist); // Filter the distances that is too far that's not meant to be
			
//			while (this.dist < BOTTOM_THRESHOLD) { // It's seeing the wall during the turn
//
//				FinalProject.usSensor.fetchSample(FinalProject.sample, 0);
//				this.dist = FinalProject.sample[0] * 100; // update distance from wall
//
//				if (this.dist > BOTTOM_THRESHOLD) {
//					FIRST_ANGLE = odometer.getTheta(); // Stop the motos when it doesn't see the wall anymore
//					Sound.buzz();
//					FinalProject.leftMotor.stop(true);
//					FinalProject.rightMotor.stop(false);
//				}
//			}

			// Save the angle read by the odometer
//			Sound.buzz();

			//navigation.turn(-360); // Rotate back to facing the wall and rotate until it sees no wall
			//Sound.buzz();

			// Pause to not mix results
//			try {
//				Thread.sleep(5000);
//			} catch (InterruptedException e) {
//				// there is nothing to be done here because it is not expected
//				// that
//				// the odometer will be interrupted by another thread
//			}

//			FinalProject.usSensor.fetchSample(FinalProject.sample, 0);
//			this.dist = FinalProject.sample[0] * 100; // update distance from wall

//			filter_far(dist); // Filter distances too big that's not meant to be

			// Rotate until it sees no wall
//			while (this.dist < BOTTOM_THRESHOLD) { // sees the wall //Using the distance in between because it can't be
//													// sure //whether it is going to be a raising or falling edge
//				FinalProject.usSensor.fetchSample(FinalProject.sample, 0);
//				this.dist = FinalProject.sample[0] * 100; // update distance from wall
//
//				if (this.dist > BOTTOM_THRESHOLD) { // There's no more wall
//													// Using the distance in between because it can't be sure
//													// whether it is going to be a raising or falling edge
//					Sound.buzz();
//					Sound.playTone(1000, 100);
//					FinalProject.leftMotor.stop(true);
//					FinalProject.rightMotor.stop(false);
//				}
//			}
//
//			SECOND_ANGLE = odometer.getTheta(); // Save the angle read by the odometer
//			updateAngle(FIRST_ANGLE, SECOND_ANGLE); // turn to face (0 axis)
		}

		// Not facing the wall
		// The robot should turn until it sees a wall
		else if (localizationType == LocalizationType.FALLING_EDGE) {

			Test.leftMotor.forward(); // Starts turning
			Test.rightMotor.backward();

			Test.usSensor.fetchSample(Test.sample, 0); // Fetch data
			double dist = Test.sample[0] * 100;

//			filter_close(dist); // Filter out distances that are too close that's not meant to be

			while (this.dist > TOP_THRESHOLD) { // Doesn't see the wall

				Test.usSensor.fetchSample(Test.sample, 0);
				this.dist = Test.sample[0] * 100; // update distance from wall

//				if (this.dist < TOP_THRESHOLD) {
//					FIRST_ANGLE = odometer.getTheta();
////					FinalProject.leftMotor.stop(true);
////					FinalProject.rightMotor.stop(false);// When it sees the wall, stop
//					Sound.buzz();
//
//				}

			}

			// Save the angle read by the odometer

//			navigation.turn(-90); // Rotate until the robot sees no wall
//			Sound.buzz();
//			Test.leftMotor.backward();
//			Test.rightMotor.forward();
//			try {
//				Thread.sleep(2500);
//			} catch (InterruptedException e) {
//				// there is nothing to be done here because it is not expected
//				// that
//				// the odometer will be interrupted by another thread
//			}

//			Test.usSensor.fetchSample(Test.sample, 0);
//			this.dist = Test.sample[0] * 100; // update distance from wall
//
//			filter_close(dist); // Filter out distances that are too close for no reasons
//
//			// Rotate until it sees a wall
//			while (this.dist > TOP_THRESHOLD) { // Doesn't see the wall
//
//				Test.usSensor.fetchSample(Test.sample, 0);
//				this.dist = Test.sample[0] * 100; // update distance from wall
//
//				if (this.dist < TOP_THRESHOLD) {
//					SECOND_ANGLE = odometer.getTheta();
//					Test.leftMotor.stop(true);
//					Test.rightMotor.stop(false); // Sees the wall
//					Sound.buzz();
//					Sound.playTone(1000, 100);
//
//				}
//			}

			// Save the angle read by the odometer
//			updateAngle(FIRST_ANGLE, SECOND_ANGLE); // turn to face (0 axis)

		}

	}

	/**
	 * Update angle. Update angle to the correct heading that we need to adjust to
	 *
	 * @param first_angle
	 *            the first angle
	 * @param second_angle
	 *            the second angle
	 */
//	public void updateAngle(double first_angle, double second_angle) {
//
//		// Computes the correct angle to turn using the the angles collected
//		if (first_angle < second_angle) {
//			UPDATED_ANGLE = 45 - ((first_angle + second_angle) / 2);
//			navigation.turn(UPDATED_ANGLE);
//		}
//
//		if (first_angle > second_angle) {
//			UPDATED_ANGLE = 225 - ((first_angle + second_angle) / 2);
//			navigation.turn(UPDATED_ANGLE);
//		}
//
//		if (localizationType == LocalizationType.FALLING_EDGE) {
//			UPDATED_ANGLE = UPDATED_ANGLE + 2; // This value is found experimentally
//			navigation.turn(UPDATED_ANGLE);
//		}
//
//		else if (localizationType == LocalizationType.RISING_EDGE) {
//			UPDATED_ANGLE = UPDATED_ANGLE + 200; // This value is found experimentally
//			navigation.turn(UPDATED_ANGLE);
//		}
//	}

	/**
	 * Convert distance.
	 *
	 * @param radius
	 *            the radius
	 * @param distance
	 *            the distance
	 * @return the int
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
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
