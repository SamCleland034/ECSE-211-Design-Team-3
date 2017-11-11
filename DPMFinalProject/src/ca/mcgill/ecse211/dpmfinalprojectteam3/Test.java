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
//		Navigation.turn(1080);
//		Button.waitForAnyPress();
		
//		Navigation.travelToWithoutAvoid(0,6);
//		try {
//			Thread.sleep(500);
//		} catch (InterruptedException e) {
//		}
//		Navigation.travelToWithoutAvoid(0,0);
		
		Navigation.travelToWithoutAvoid(0, 2);
		Navigation.travelToWithoutAvoid(4, 2);
		Sound.beepSequenceUp();
		Navigation.travelToWithoutAvoid(4, 0);
		Sound.beepSequenceUp();
		Navigation.travelToWithoutAvoid(0, 0);
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
		Avoidance master = new Avoidance(gps);
		LinkedList<Integer> coordsList = new LinkedList<Integer>();
		FinalProject.odometer.setX(0);
		FinalProject.odometer.setY(0);
		coordsList.addLast(0);
		coordsList.addLast(2);
		gps.setPath(coordsList);
		
		OdometryCorrection oc = new OdometryCorrection(FinalProject.odometer, leftpoller, rightpoller, jointpoller);
		oc.setNavigation(gps);
		gps.setOdometryCorrection(oc);
		gps.setAvoidance(master);
		jointpoller.on();
		leftpoller.on();
		rightpoller.on();
		jointpoller.start();
		leftpoller.start();
		rightpoller.start();
		oc.on();
		master.on();
		//master.start();
		oc.start();
		gps.startNav();
		Sound.beepSequenceUp();
		oc.off();
		
		coordsList.clear();
		coordsList.addLast(4);
		coordsList.addLast(2);
		gps.setPath(coordsList);
		oc.on();
		gps.startNav();
		Sound.beepSequenceUp();
		oc.off();
		
		coordsList.clear();
		coordsList.addLast(4);
		coordsList.addLast(0);
		gps.setPath(coordsList);
		oc.on();
		gps.startNav();
		Sound.beepSequence();
		oc.off();
		coordsList.clear();
		coordsList.addLast(0);
		coordsList.addLast(0);
		gps.setPath(coordsList);
		oc.on();
		//Sound.beepSequence();
		gps.startNav();
		Sound.beepSequence();
		oc.off();
		//Navigation.turnTo(0);
		
		
		

	}
}
