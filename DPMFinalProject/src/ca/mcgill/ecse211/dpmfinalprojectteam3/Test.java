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
	private static final Port LightPort = LocalEV3.get().getPort("S4");
	static SampleProvider colorProvider;
	public static double[] RGBColors;
	public static  Navigation gps;
	public static Avoidance master;
	public static UltrasonicPoller uspoller;
	public static LightPoller colorpoller;
	public static LinkedList<Integer> searchList ;
	public static SensorRotation sensorMotor ;
	public static JointLightPoller jointpoller;
	public static OdometryCorrection oc;
	
	
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
		
//		Navigation.travelToWithoutAvoid(0, 2);
//		Navigation.travelToWithoutAvoid(4, 2);
//		Sound.beepSequenceUp();
//		Navigation.travelToWithoutAvoid(4, 0);
//		Sound.beepSequenceUp();
//		Navigation.travelToWithoutAvoid(0, 0);
	}
	public static void UltrasonicTest() {
		Navigation gps = new Navigation(FinalProject.odometer);
		LocalizationType lt = LocalizationType.RISINGEDGE;
		Avoidance master = new Avoidance(gps);
		UltrasonicPoller uspoller = new UltrasonicPoller(usDist, sample, master, gps);
		UltrasonicLocalizer usLoc = new UltrasonicLocalizer(FinalProject.odometer, gps, lt, uspoller);
		LightPoller leftpoller = new LightPoller(FinalProject.leftSensor, FinalProject.leftProvider);
		LightPoller rightpoller = new LightPoller(FinalProject.rightSensor, FinalProject.rightProvider);
		JointLightPoller jointpoller = new JointLightPoller(FinalProject.leftProvider, FinalProject.rightProvider);
		uspoller.on();
		uspoller.start();
		//master.start();
		
		usLoc.doLocalization();
		while (usLoc.localizing) {
//			continue;
		}
		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {
		}
//		Button.waitForAnyPress();
		LightLocalizer lightLoc = new LightLocalizer(FinalProject.odometer, gps, leftpoller, rightpoller, jointpoller);
		uspoller.off();
		jointpoller.on();
		leftpoller.on();
		rightpoller.on();
		jointpoller.start();
		leftpoller.start();
		rightpoller.start();
		lightLoc.startLightLOC4();
		
		
		jointpoller.off();
		leftpoller.off();
		rightpoller.off();
		
		LinkedList<Double> coordsList = new LinkedList<Double>();
		FinalProject.odometer.setX(0);
		FinalProject.odometer.setY(0);
		coordsList.addLast(0.0);
		coordsList.addLast(2.0);
		gps.setPath(coordsList);
		
		OdometryCorrection oc = new OdometryCorrection(FinalProject.odometer, leftpoller, rightpoller, jointpoller);
		oc.setNavigation(gps);
		gps.setOdometryCorrection(oc);
		gps.setAvoidance(master);
		
		jointpoller.on();
		leftpoller.on();
		rightpoller.on();
		oc.on();
		master.on();
		//master.start();
		oc.start();
		gps.startNav();
		oc.off();
		
		coordsList.clear();
		coordsList.addLast(1.0);
		coordsList.addLast(2.0);
		gps.setPath(coordsList);
		oc.on();
		gps.startNav();
		
		
		
		gps.zipTraversal();
	
	}
	public static void CorrectionTest() {
		Navigation gps = new Navigation(FinalProject.odometer);
		LightPoller leftpoller = new LightPoller(FinalProject.leftSensor, FinalProject.leftProvider);
		LightPoller rightpoller = new LightPoller(FinalProject.rightSensor, FinalProject.rightProvider);
		JointLightPoller jointpoller = new JointLightPoller(FinalProject.leftProvider, FinalProject.rightProvider);
		Avoidance master = new Avoidance(gps);
		LinkedList<Double> coordsList = new LinkedList<Double>();
		FinalProject.odometer.setX(0);
		FinalProject.odometer.setY(0);
		coordsList.addLast(0.0);
		coordsList.addLast(2.0);
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
		coordsList.addLast(4.0);
		coordsList.addLast(2.0);
		gps.setPath(coordsList);
		oc.on();
		gps.startNav();
		Sound.beepSequenceUp();
		oc.off();
		
		coordsList.clear();
		coordsList.addLast(4.0);
		coordsList.addLast(0.0);
		gps.setPath(coordsList);
		oc.on();
		gps.startNav();
		Sound.beepSequence();
		oc.off();
		coordsList.clear();
		coordsList.addLast(0.0);
		coordsList.addLast(0.0);
		gps.setPath(coordsList);
		oc.on();
		//Sound.beepSequence();
		gps.startNav();
		Sound.beepSequence();
		oc.off();
		//Navigation.turnTo(0);
	}
	
	public static void FlagTest() {
		colorProvider = FinalProject.colorSensor.getRGBMode();
		colorpoller = new LightPoller(FinalProject.colorSensor, colorProvider);
		gps = new Navigation(FinalProject.odometer);
		master = new Avoidance(gps);
		uspoller = new UltrasonicPoller(usDist, sample, master, gps);
		searchList = new LinkedList<Integer>();
		sensorMotor = new SensorRotation(master, FinalProject.usMotor, gps);
		jointpoller = new JointLightPoller(FinalProject.leftProvider, FinalProject.rightProvider);
		oc = new OdometryCorrection(FinalProject.odometer, null, null, jointpoller);
		uspoller.on();
		uspoller.start();
		oc.start();
		oc.off();
		colorpoller.on();
		colorpoller.start();
		colorpoller.off();
		sensorMotor.on();
		jointpoller.on();

		jointpoller.start();
	
		gps.setOdometryCorrection(oc);
		gps.setAvoidance(master);	
		master.on();
//		sensorMotor.start();
//		searchList.addLast(0);
//		searchList.addLast(1);
//		searchList.addLast(0);
//		searchList.addLast(3);
//		searchList.addLast(2);
//		searchList.addLast(3);
//		searchList.addLast(2);
//		searchList.addLast(1);
		searchList.addLast(1);
		searchList.addLast(1);
		searchList.addLast(4);
		searchList.addLast(4);
		
		gps.setSearchRegionPath(searchList);
		
		
		FinalProject.odometer.setX(1 * FinalProject.TILE_SPACING);
		FinalProject.odometer.setY(1 * FinalProject.TILE_SPACING);
		FinalProject.odometer.setTheta(0);
		
	
		int color =1;
		gps.flagSearchTravel(color);
		
		
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
}