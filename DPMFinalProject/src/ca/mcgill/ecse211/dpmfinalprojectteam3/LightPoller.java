package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

// 
/**
 * Class used for sampling data for the light sensors, particularly the color
 * sensor that will be used for color detection. The other two sensors for line
 * detection will use the JointLightPoller class to synchronize their data with
 * each other. Will be terminated once the robot has found the flag
 */
public class LightPoller extends Thread {

	/** The sensor. */
	private EV3ColorSensor sensor;

	/** The provider. */
	private SampleProvider provider;

	/** The sample. */
	private float[] sample;

	/** The light val. */
	private double lightVal = 0;
	private static double[] colorValues = new double[3];

	/** The last light val. */
	private double lastLightVal = 0;

	/** The on. */
	public static boolean on;

	/**
	 * Instantiates a new light poller.
	 *
	 * @param sensor
	 *            the sensor
	 * @param provider
	 *            the provider
	 */
	public LightPoller(EV3ColorSensor sensor, SampleProvider provider) {
		this.sensor = sensor;
		this.provider = provider;
		on = false;

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		sample = new float[provider.sampleSize()];
		while (true) {
			if (on) {
				synchronized (this) {
					provider.fetchSample(sample, 0); // acquire data
					colorValues[0] = sample[0];// extract from buffer, cast to int
					colorValues[1] = sample[1];
					colorValues[2] = sample[2];
				}
				try {
					Thread.sleep(50);
				} catch (Exception e) {
				} // Poor man's timed sampling
			} else {
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
				}
			}
		}
	}

	/**
	 * Gets the change in light.
	 *
	 * @return the change in light
	 */
	public double getChangeInLight() {
		double result;
		synchronized (this) {
			result = (lightVal - lastLightVal) / lastLightVal;
		}
		return result;
	}

	/**
	 * Gets the light val.
	 *
	 * @return the light val
	 */
	public double[] getLightVal() {
		double[] result;
		synchronized (this) {
			result = colorValues;
		}

		return result;
	}

	/**
	 * Gets the last light val.
	 *
	 * @return the last light val
	 */
	public double getLastLightVal() {
		double result;
		synchronized (this) {
			result = lastLightVal;
		}
		return result;
	}

	/**
	 * turn thread on.
	 */
	public static void on() {
		on = true;
	}

	/**
	 * turn thread off.
	 */
	public static void off() {
		on = false;
	}

	public boolean checkRed() {
		boolean result = false;
		synchronized (this) {
			result = (colorValues[0] > colorValues[1] && colorValues[0] > colorValues[2] && colorValues[1] != colorValues[2] && Math.abs(colorValues[0]-colorValues[1])>Math.abs(colorValues[1]-colorValues[2]));
		}	
			System.out.println(colorValues[0]);
			System.out.println(colorValues[1]);
			System.out.println(colorValues[2]);
		return result;
	}

	public boolean checkBlue() {
		boolean result = false;
		synchronized (this) {
			result = (colorValues[0] < colorValues[2]);
				
		}
			System.out.println(colorValues[0]);
			System.out.println(colorValues[1]);
			System.out.println(colorValues[2]);
		return result;
	}

	public boolean checkWhite() {//only works for white for some reason
		boolean result = false;
		synchronized (this) {
			result = colorValues[0] > colorValues[1] && colorValues[1] > colorValues[2]&& (2.2*(Math.abs((colorValues[0]-colorValues[1])))<((Math.abs(colorValues[1]-colorValues[2]))))&&colorValues[0]<1;
			System.out.println("R :" +colorValues[0]);
			System.out.println("G :" +colorValues[1]);
			System.out.println("B :" +colorValues[2]);
		}
		
		return result;
	}
	public boolean checkYellow() {
		boolean result = false;
		synchronized (this) {
			result = colorValues[0] > colorValues[1] && colorValues[1] > colorValues[2]  && colorValues[2] < 0.002;
			System.out.println("R :" +colorValues[0]);
			System.out.println("G :" +colorValues[1]);
			System.out.println("B :" +colorValues[2]);
		}
		return result;
	}

	public boolean checkColors(int correctColor) {
		boolean result = false;
		//synchronized (this) {
			/*
			 * result = (colorValues[0] < FinalProject.RGBColors[0] + FinalProject.EPSILON
			 * && colorValues[0] > FinalProject.RGBColors[0] - FinalProject.EPSILON) &&
			 * (colorValues[1] < FinalProject.RGBColors[1] + FinalProject.EPSILON &&
			 * colorValues[1] > FinalProject.RGBColors[1] - FinalProject.EPSILON) &&
			 * (colorValues[2] < FinalProject.RGBColors[2] + FinalProject.EPSILON &&
			 * colorValues[2] > FinalProject.RGBColors[2] - FinalProject.EPSILON);
			 * 
			 * if (colorValues[0] > 0.45 && colorValues[1] > 0.45 && colorValues[2] > 0.098)
			 * result = 4; else if (colorValues[0] > colorValues[1] && colorValues[0] >
			 * colorValues[2]) result = 1; else if (colorValues[0] < colorValues[2] &&
			 * colorValues[1] < colorValues[2]) result = 2; else if (colorValues[0] >
			 * colorValues[1] && colorValues[1] > colorValues[2]) result = 3;
			 */
			if (correctColor == 1)
				result = checkRed();
			else if (correctColor == 2)
				result = checkBlue();
			else if (correctColor == 3)
				result = checkYellow();
			else if (correctColor == 4)
				result = checkWhite();
	//	}
		return result;
	}
}
