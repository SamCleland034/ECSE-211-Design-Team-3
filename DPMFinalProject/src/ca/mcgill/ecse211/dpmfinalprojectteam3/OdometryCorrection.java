
package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;

/**
 * The Class OdometryCorrection, used to correct the small but accumulating
 * errors of the odometer using the light sensor to detect gridlines on the
 * floor
 * 
 * @version 1.0
 */
public class OdometryCorrection extends Thread {

	/** The odometer. */
	private Odometer odometer;
	private LightPoller leftPoller;
	private LightPoller rightPoller;

	private boolean on;
	private JointLightPoller jointPoller;
	private static int SAMPLINGPERIOD = 15;
	/** The distance between lines. */
	private static double TILE_SPACING = 30;
	public boolean corrected = false;
	private Navigation gps;
	public static int counter2 = 0;


	// private EV3ColorSensor colorSensor;

	private static final double SENSOR_OFFSET = 12.9;

	// constructor

	/**
	 * Instantiates a new odometry correction.
	 *
	 * @param odometer
	 *            the odometer
	 */
	public OdometryCorrection(Odometer odometer, LightPoller leftPoller, LightPoller rightPoller,
			JointLightPoller jointPoller) {

		this.odometer = odometer;
		this.leftPoller = leftPoller;
		this.rightPoller = rightPoller;
		this.jointPoller = jointPoller;
		this.on = false;
	}

	// run method (required for Thread)

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		int speed = 0;
		long startTime = 0;
//		long endTime;
//		double[] lightValue;
		int counter = 0;
		counter2 = 0; 
		double LeftLightValue;
		double RightLightValue;
//		long RightTime;
//		long LeftTime;
		
		while (true) {
			//if (on) {

				startTime = System.currentTimeMillis();				
				//while(Navigation.corrected == false){
					if(Navigation.isTurning == false) {
						LeftLightValue = jointPoller.getLeftValue();
						RightLightValue = jointPoller.getRightValue();
						
						if (LeftLightValue < 0.23 && RightLightValue < 0.23) {
							if (counter2 == 0){

								Sound.beepSequence();
								checkOrientation();
								counter = 0;
								counter2 = 1;
								Navigation.corrected = true;
								sleepFor(1);
							}
							else {
								counter2--;
								checkOrientation();
								sleepFor(2);
							}


						} 
						else if (counter > 2000) { //counter value is high because it increments very fast
							if(LeftLightValue < 0.23 && RightLightValue > 0.23 ) {
								if(counter2 == 0) {
									//							LeftTime = System.currentTimeMillis();
									//							while(RightLightValue > 0.23) {
									//			
									//							}
									//							RightTime = System.currentTimeMillis();
									//
									//							if ( RightTime-LeftTime> 0.01) {
									FinalProject.rightMotor.stop(true);
									FinalProject.leftMotor.stop(false);

									speed = FinalProject.leftMotor.getSpeed();

									Sound.buzz();

									checkRightPoller(speed);
									counter = 0;
									counter2 = 1;
									Navigation.corrected = true;
									sleepFor(1);
								}
								else counter2--;
								checkOrientation();
								sleepFor(2);
								//							}
								//							else checkOrientation();
								//							Navigation.corrected = true;
							}
							else if (RightLightValue < 0.23 && LeftLightValue > 0.23) {
								if(counter2 == 0) {
									//							RightTime = System.currentTimeMillis();
									//							while(RightLightValue > 0.23) {
									//						
									//							}
									//							LeftTime = System.currentTimeMillis();
									//
									//							if (LeftTime-RightTime > 0.01) {
									FinalProject.leftMotor.stop(true);
									FinalProject.rightMotor.stop(false);

									speed = FinalProject.leftMotor.getSpeed();

									Sound.beep();
									checkLeftPoller(speed);
									counter = 0;
									counter2 = 1;
									Navigation.corrected = true;
									sleepFor(1);
								}
								else counter2--;
								checkOrientation();
								sleepFor(2);
								//							}
								//							else checkOrientation();
								//							Navigation.corrected = true;
							}
						}
						else counter++;
					}
				}
				//			endTime = System.currentTimeMillis();
				//			if (endTime - startTime < SAMPLINGPERIOD) {
				//				try {
				//					Thread.sleep((SAMPLINGPERIOD - (endTime - startTime)));
				//				} catch (InterruptedException e) {
				//				}
				//			}
				//			else {
				//				try {
				//					Thread.sleep(1000);
				//				} catch (InterruptedException e) {
				//				}
				//			}

			//}
		//}
	}

	private void checkOrientation() {
		double theta;
		int correctedX = 0;
		int correctedY = 0;	

		theta = odometer.getTheta();
		if ((theta) > 7 * Math.PI / 4 || ((theta > 0) && (theta <= Math.PI / 4))) {
			odometer.setTheta(0);
			correctedY = (int) (odometer.getY() / TILE_SPACING);
			odometer.setY(correctedY * TILE_SPACING + SENSOR_OFFSET);
		} else if (theta >= 5 * Math.PI / 4 && theta <= 7 * Math.PI / 4) {
			odometer.setTheta(3 * Math.PI / 2);
			correctedY = (int) (odometer.getY() / TILE_SPACING);
			odometer.setY(correctedY * TILE_SPACING + SENSOR_OFFSET);
		} else if (theta >= 3 * Math.PI / 4 && theta <= 5 * Math.PI / 4) {
			odometer.setTheta(Math.PI);
			correctedX = (int) (odometer.getX() / TILE_SPACING);
			odometer.setX(correctedX * TILE_SPACING + SENSOR_OFFSET);
		} else {
			odometer.setTheta(Math.PI / 2);
			correctedX = (int) (odometer.getX() / TILE_SPACING);
			odometer.setX(correctedX * TILE_SPACING + SENSOR_OFFSET);

		}
	}

	private void checkRightPoller(int speed) {
		
		float right_speed = (float) (speed * Navigation.RIGHT_OFFSET);	
	
		long startTime = System.currentTimeMillis();
		
		FinalProject.rightMotor.setSpeed(45);
		FinalProject.leftMotor.setSpeed(45);
		
		while (jointPoller.getLeftValue()>0.23) {
			
			FinalProject.leftMotor.backward();
			FinalProject.rightMotor.backward();
		}
		FinalProject.leftMotor.stop(true);
		FinalProject.rightMotor.stop(false);
		
		FinalProject.rightMotor.setSpeed(50);
		
		while (jointPoller.getRightValue() > 0.23) {
			
			FinalProject.rightMotor.forward();
			//			if (timedOut(startTime))
			//				break;
			//			continue;
		}
		FinalProject.rightMotor.stop(false);
		checkOrientation();

		FinalProject.leftMotor.setSpeed(speed);
		FinalProject.rightMotor.setSpeed(right_speed);
		

	}

	private boolean timedOut(long startTime) {
		if (System.currentTimeMillis() - startTime > 50) {
			return true;
		} else  return false;
	}

	private void checkLeftPoller(int speed) {
		
		float right_speed = (float) (speed * Navigation.RIGHT_OFFSET);	

		long startTime = System.currentTimeMillis();
		
		FinalProject.rightMotor.setSpeed(45);
		FinalProject.leftMotor.setSpeed(45);
		
		while (jointPoller.getRightValue()>0.23) {
						
			FinalProject.leftMotor.backward();
			FinalProject.rightMotor.backward();
		}
		FinalProject.leftMotor.stop(true);
		FinalProject.rightMotor.stop(false);
		
		FinalProject.leftMotor.setSpeed(50);
		
		while (jointPoller.getLeftValue() > 0.23) {

			
			FinalProject.leftMotor.forward();
			//			if (timedOut(startTime))
			//				break;
			//			continue;
		}
		FinalProject.leftMotor.stop(false);
		checkOrientation();
		FinalProject.leftMotor.setSpeed(speed);
		FinalProject.rightMotor.setSpeed(right_speed);
	

	}

	public void setNavigation(Navigation gps) {
		this.gps = gps;
	}

	public void on() {
		this.on = true;
	}

	public void off() {
		this.on = false;
	}
	private void sleepFor(int i) {
		try {
			sleep(1000 * i);
		} catch (InterruptedException e) {
		}
	}
}
