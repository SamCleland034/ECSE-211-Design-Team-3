
package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer extends Thread {
	
	//Define the two types of correction
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };

	//Define variables
	public static float ROTATE_SPEED = 70;
	private double distance;
	private double dist;
	private double UPDATED_ANGLE;
	//The noise margin is set to be 35 to 55 which is determined experimentally
	private static final int TOP_THRESHOLD = 51;	// "d+k"
	private static final int BOTTOM_THRESHOLD = 35; // "d-k"
	private static final int MAX_DISTANCE_BETWEEN = 43; // "d"
	//Used to filter values
	private static final int FILTER_OUT = 20;
	
	private Odometer odometer;
	private Navigation navigation;
	private LocalizationType localizationType;
	private int filterControl;

	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			Navigation navigation, LocalizationType localizationType) {
		this.odometer = odometer;
		this.localizationType = localizationType;
		this.navigation = navigation;

		// Reset the motors
		for (EV3LargeRegulatedMotor motors : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motors.stop();
			motors.setAcceleration(500);
		}
	}

	public void run() {

		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}
		
		doLocalize(); //Start localizing
	}

	public void doLocalize() {

		double FIRST_ANGLE, SECOND_ANGLE; // To store two angles when it changes from seeing the wall to facing
										  // away or vice versa 
	
		if (localizationType == LocalizationType.RISING_EDGE) { //Starts by facing the wall

			navigation.turn(360); //Turns
			
			Lab5.usSensor.fetchSample(Lab5.sample, 0); // Get data from ultrasonic sensor
			double dist = Lab5.sample[0] * 100;

			filter_far(dist); // Filter the distances that is too far that's not meant to be

			while (this.dist < BOTTOM_THRESHOLD) { // It's seeing the wall during the turn

				Lab5.usSensor.fetchSample(Lab5.sample, 0);
				this.dist = Lab5.sample[0] * 100; // update distance from wall

				if (this.dist > BOTTOM_THRESHOLD) { // Stop the motos when it doesn't see the wall anymore
					Sound.buzz();
					Lab5.leftMotor.stop();
					Lab5.rightMotor.stop();
				}
			}

			FIRST_ANGLE = odometer.getTheta(); //Save the angle read by the odometer
			Sound.buzz();

			navigation.turn(-360); // Rotate back to facing the wall and rotate until it sees no wall
			Sound.buzz();

			// Pause to not mix results
			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {
				// there is nothing to be done here because it is not expected
				// that
				// the odometer will be interrupted by another thread
			}

			Lab5.usSensor.fetchSample(Lab5.sample, 0);
			this.dist = Lab5.sample[0] * 100; // update distance from wall

			filter_far(dist); // Filter distances too big that's not meant to be

			// Rotate until it sees no wall
			while (this.dist < BOTTOM_THRESHOLD) {	// sees the wall
														//Using the distance in between because it can't be sure
														//whether it is going to be a raising or falling edge

				Lab5.usSensor.fetchSample(Lab5.sample, 0);
				this.dist = Lab5.sample[0] * 100; // update distance from wall

				if (this.dist > BOTTOM_THRESHOLD) { //There's no more wall
														//Using the distance in between because it can't be sure
														//whether it is going to be a raising or falling edge
					Sound.buzz();
					Sound.playTone(1000, 100);
					Lab5.leftMotor.stop();
					Lab5.rightMotor.stop();
				}
			}
			
			SECOND_ANGLE = odometer.getTheta(); //Save the angle read by the odometer
			updateAngle(FIRST_ANGLE, SECOND_ANGLE); // turn to face (0 axis)
			
		}

		// Not facing the wall
		// The robot should turn until it sees a wall
		else if (localizationType == LocalizationType.FALLING_EDGE) { 

			navigation.turn(360); //Starts turning

			Lab5.usSensor.fetchSample(Lab5.sample, 0); //Fetch data
			double dist = Lab5.sample[0] * 100;

			filter_close(dist); //Filter out distances that are too close that's not meant to be

			while (this.dist > TOP_THRESHOLD) { // Doesn't see the wall

				Lab5.usSensor.fetchSample(Lab5.sample, 0);
				this.dist = Lab5.sample[0] * 100; // update distance from wall

				if (this.dist < TOP_THRESHOLD) { // When it sees the wall, stop
					Sound.buzz();
					Lab5.leftMotor.stop();
					Lab5.rightMotor.stop();
				}

			}

			FIRST_ANGLE = odometer.getTheta(); //Save the angle read by the odometer
			Sound.buzz();

			navigation.turn(-360); // Rotate until the robot sees no wall
			Sound.buzz();

			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {
				// there is nothing to be done here because it is not expected
				// that
				// the odometer will be interrupted by another thread
			}

			Lab5.usSensor.fetchSample(Lab5.sample, 0);
			this.dist = Lab5.sample[0] * 100; // update distance from wall

			filter_close(dist); //Filter out distances that are too close for no reasons

			// Rotate until it sees a wall
			while (this.dist > TOP_THRESHOLD) { // Doesn't see the wall

				Lab5.usSensor.fetchSample(Lab5.sample, 0);
				this.dist = Lab5.sample[0] * 100; // update distance from wall

				if (this.dist < TOP_THRESHOLD) { // Sees the wall
					Sound.buzz();
					Sound.playTone(1000, 100);
					Lab5.leftMotor.stop();
					Lab5.rightMotor.stop();
					}
			}

			SECOND_ANGLE = odometer.getTheta(); //Save the angle read by the odometer
			updateAngle(FIRST_ANGLE, SECOND_ANGLE); // turn to face (0 axis)
			
		}

	}

	public void updateAngle(double first_angle, double second_angle) {
		
		//Computes the correct angle to turn using the the angles collected
		if (first_angle < second_angle) {
		UPDATED_ANGLE = 45 - ((first_angle + second_angle) / 2);
		navigation.turn(UPDATED_ANGLE);
		}
		
		if (first_angle > second_angle) {
			UPDATED_ANGLE = 225 - ((first_angle + second_angle) / 2);
			navigation.turn(UPDATED_ANGLE);
		}
		
		
		if (localizationType == LocalizationType.FALLING_EDGE) {
			UPDATED_ANGLE = UPDATED_ANGLE + 29; //This value is found experimentally
			navigation.turn(UPDATED_ANGLE);
		}

		else if (localizationType == LocalizationType.RISING_EDGE) {
			UPDATED_ANGLE = UPDATED_ANGLE + 200; //This value is found experimentally
			navigation.turn(UPDATED_ANGLE);
		} 
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
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
	
	//Filter out distances that are too close for no reasons
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
