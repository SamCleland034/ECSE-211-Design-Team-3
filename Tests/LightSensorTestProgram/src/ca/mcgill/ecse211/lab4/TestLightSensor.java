package ca.mcgill.ecse211.lab4;

import java.util.ArrayList;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class TestLightSensor extends Thread {
	private ArrayList<Double> reddata;
	private ArrayList<Double> greendata;
	private ArrayList<Double> bluedata;
	private int correctionPeriod = 30;
	private EV3ColorSensor sensor;
	private long threadStart;
	private long threadDone;
	private float[] lightData;
	private SampleProvider provider;
	private double sample;
	private int counter = 0;
	public boolean done;

	public TestLightSensor(EV3ColorSensor lssensor, SampleProvider provider) {
		this.sensor = lssensor;
		this.provider = provider;
		this.lightData = new float[3];
		sample = 0;
		this.reddata = new ArrayList<Double>();
		this.bluedata = new ArrayList<Double>();
		this.greendata = new ArrayList<Double>();
		this.done = false;
	}

	public void start() {
		while (counter < 500) {
			threadStart = System.currentTimeMillis();
			for (int i = 0; i < provider.sampleSize(); i++) {
				provider.fetchSample(lightData, 0);
			}
			double sampleR = 100 * (double) lightData[0];
			double sampleG = 100 * (double) lightData[1];
			double sampleB = 100 * (double) lightData[2];
			reddata.add(sampleR);
			bluedata.add(sampleB);
			greendata.add(sampleG);
			System.out.println(sampleG);
			threadDone = System.currentTimeMillis();
			if (threadDone - threadStart < correctionPeriod) {
				try {
					Thread.sleep(correctionPeriod - (threadDone - threadStart));
				} catch (InterruptedException e) {
				}
			}
			counter++;
		}
		TestingLab.leftMotor.stop(true);
		TestingLab.rightMotor.stop(false);
		double sum = 0;
		double mean = 0;
		double stDev = 0;
		for (int i = 0; i < reddata.size(); i++) {
			sum += reddata.get(i);
		}
		mean = sum / reddata.size();
		sum = 0;
		for (int i = 0; i < reddata.size(); i++) {
			sum += Math.pow(reddata.get(i) - mean, 2);
		}
		stDev = Math.sqrt(sum / (reddata.size() - 1));
		System.out.println("Mean red=" + mean);
		System.out.println("Standard Dev red=" + stDev);
		sum = 0;
		mean = 0;
		stDev = 0;
		for (int i = 0; i < greendata.size(); i++) {
			sum += greendata.get(i);
		}
		mean = sum / greendata.size();
		sum = 0;
		for (int i = 0; i < greendata.size(); i++) {
			sum += Math.pow(greendata.get(i) - mean, 2);
		}
		stDev = Math.sqrt(sum / (greendata.size() - 1));
		System.out.println("Mean green=" + mean);
		System.out.println("Standard Dev green=" + stDev);
		sum = 0;
		mean = 0;
		stDev = 0;
		for (int i = 0; i < bluedata.size(); i++) {
			sum += bluedata.get(i);
		}
		mean = sum / bluedata.size();
		sum = 0;
		for (int i = 0; i < bluedata.size(); i++) {
			sum += Math.pow(bluedata.get(i) - mean, 2);
		}
		stDev = Math.sqrt(sum / (bluedata.size() - 1));
		System.out.println("Mean blue=" + mean);
		System.out.println("Standard Dev blue=" + stDev);
		this.done = true;
	}

}
