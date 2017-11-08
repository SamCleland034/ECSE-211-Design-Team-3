package ca.mcgill.ecse211.dpmfinalprojectteam3;

import java.util.LinkedList;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;


public class Test {
	static SampleProvider usDist = FinalProject.usSensor.getMode("Distance");
	static float[] sample = new float[usDist.sampleSize()];
	
	public static void NavigationTest() {
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
		Navigation.turn(1080);
		Button.waitForAnyPress();
		
		Navigation.travelToWithoutAvoid(0,6);
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
		Navigation.travelToWithoutAvoid(0,0);
	}
	public static void UltrasonicTest() {
		Navigation gps = new Navigation(FinalProject.odometer);
		LocalizationType lt = LocalizationType.FALLINGEDGE;
		Avoidance master = new Avoidance(gps);
		UltrasonicPoller uspoller = new UltrasonicPoller(usDist, sample, master, gps);
		UltrasonicLocalizer usLoc = new UltrasonicLocalizer(FinalProject.odometer, gps, lt, uspoller);
		uspoller.on();
		uspoller.start();
		//master.start();
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {}
		usLoc.doLocalization();
		while (usLoc.localizing)
			continue;
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		}
		
	
	}
	public static void CorrectionTest() {
		Navigation gps = new Navigation(FinalProject.odometer);
		LightPoller leftpoller = new LightPoller(FinalProject.leftSensor, FinalProject.leftProvider);
		LightPoller rightpoller = new LightPoller(FinalProject.rightSensor, FinalProject.rightProvider);
		JointLightPoller jointpoller = new JointLightPoller(FinalProject.leftProvider, FinalProject.rightProvider);
		jointpoller.on();
		jointpoller.start();
		leftpoller.start();
		rightpoller.start();
		
		OdometryCorrection oc = new OdometryCorrection(FinalProject.odometer, leftpoller, rightpoller, jointpoller);
		oc.setNavigation(gps);
		gps.setOdometryCorrection(oc);
		jointpoller.on();
		oc.on();
		Navigation.travelTo(0, 6);
		gps.startNav();

	
	}
}
