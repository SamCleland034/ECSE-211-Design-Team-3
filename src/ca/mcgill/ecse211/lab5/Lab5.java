//TESTTT

package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;

public class Lab5 extends Thread {

	// Assign ports to motors and to sensor
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor zipMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
	private Odometer odometer;

	static SampleProvider usDist = usSensor.getMode("Distance");
	static float[] sample = new float[usDist.sampleSize()];

	public static final double WHEEL_RADIUS = 2.145; // radius of wheel
	public static final double TRACK = 15.15; // Width of car
	private static double origin[][] = { { 0, 0 } ,{1,1}};
	private static int x=0;
	private static int y=0;
	private static int xc=0;
	private static int yc=0;

	public static void main(String[] args) {
		int buttonChoice;

		// instantiate threads controlling the robot
		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);

		// clear the display
		t.clear();

		// ask the user whether the robot should use rising edge or falling edge
		t.drawString("Select X and Y  ", 0, 0);
		t.drawString("                ", 0, 1);
		t.drawString("                ", 0, 2);
		t.drawString("                ", 0, 3);
		t.drawString("                ", 0, 4);

		// wait for the user to press a button and start the odometer and
		// odometer display
		buttonChoice = Button.waitForAnyPress();
		

		while (buttonChoice != Button.ID_ENTER){
			zipMotor.rotate(720);

			if (buttonChoice == Button.ID_RIGHT){
				if (x < 8){
					x++;
				}
				t.clear();
				t.drawString("X:", 0, 0);
				t.drawInt(x, 3, 0);
				t.drawString("Y:", 0, 2);
				t.drawInt(y, 3, 2);
			}

			if (buttonChoice == Button.ID_LEFT){
				if (x > 0){
					x--;
				}
				t.clear();
				t.drawString("X:", 0, 0);
				t.drawInt(x, 3, 0);
				t.drawString("Y:", 0, 2);
				t.drawInt(y, 3, 2);
			}

			if (buttonChoice == Button.ID_UP){
				if (y < 8){
					y++;
				}
				t.clear();
				t.drawString("X:", 0, 0);
				t.drawInt(x, 3, 0);
				t.drawString("Y:", 0, 2);
				t.drawInt(y, 3, 2);
			}

			if (buttonChoice == Button.ID_DOWN){
				if (y > 0){
					y--;
				} 

				t.clear();
				t.drawString("X:", 0, 0);
				t.drawInt(x, 3, 0);
				t.drawString("Y:", 0, 2);
				t.drawInt(y, 3, 2);
			}

			buttonChoice = Button.waitForAnyPress();
		}


		t.clear();
		
		// ask the user whether the robot should use rising edge or falling edge
				t.drawString("Select Xc and Yc  ", 0, 0);
				t.drawString("                  ", 0, 1);
				t.drawString("                  ", 0, 2);
				t.drawString("                  ", 0, 3);
				t.drawString("                  ", 0, 4);


		buttonChoice = Button.waitForAnyPress();

		

		while (buttonChoice != Button.ID_ENTER){

			if (buttonChoice == Button.ID_RIGHT){
				if (xc < 8){
					xc++;
				}
				t.clear();
				t.drawString("Xc:", 0, 0);
				t.drawInt(xc, 3, 0);
				t.drawString("Yc:", 0, 2);
				t.drawInt(yc, 3, 2);
			}

			if (buttonChoice == Button.ID_LEFT){
				if (xc > 0){
					xc--;
				}
				t.clear();
				t.drawString("Xc:", 0, 0);
				t.drawInt(xc, 3, 0);
				t.drawString("Yc:", 0, 2);
				t.drawInt(yc, 3, 2);
			}

			if (buttonChoice == Button.ID_UP){
				if (yc < 8){
					yc++;
				}
				t.clear();
				t.drawString("Xc:", 0, 0);
				t.drawInt(xc, 3, 0);
				t.drawString("Yc:", 0, 2);
				t.drawInt(yc, 3, 2);
			}

			if (buttonChoice == Button.ID_DOWN){
				if (yc > 0){
					yc--;
				} 

				t.clear();
				t.drawString("Xc:", 0, 0);
				t.drawInt(xc, 3, 0);
				t.drawString("Yc:", 0, 2);
				t.drawInt(yc, 3, 2);
			}
			buttonChoice = Button.waitForAnyPress();
		}
		
		double coordinate[][] = {{x,y},{xc,yc}};

		if (buttonChoice == Button.ID_ENTER) {

			odometer.start();
			odometryDisplay.start();	
			Navigation navigation = new Navigation(odometer, coordinate, leftMotor, rightMotor);
			navigation.start();
			//			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, navigation,
			//					UltrasonicLocalizer.LocalizationType.RISING_EDGE);
			//			usLocalizer.start();

			// wait for the user to press any button and start light localizer
			// thread

			//			Button.waitForAnyPress();



			//			LightLocalizer lightLocalizer = new LightLocalizer(odometer, navigation);
			//			lightLocalizer.start();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}
