package ca.mcgill.ecse211.dpmfinalprojectteam3;

public class Avoidance extends Thread {
	private Navigation gps;
	private static final long SAMPLINGPERIOD = 50;
	private UltrasonicPoller poller;
	private static final int FILTERCONTROL = 100;
	public boolean avoiding;
	public boolean inDanger;
	private SensorRotation sensorMotor;

	public Avoidance(Navigation gps) {
		this.gps = gps;
		this.avoiding = false;
		gps.setAvoidance(this);
		this.inDanger = false;
	}

	public void setSensorRotation(SensorRotation sensorMotor) {
		this.sensorMotor = sensorMotor;
	}

	public void setPoller(UltrasonicPoller poller) {
		this.poller = poller;
	}

	public void run() {
		int distance;
		long startTime, endTime;
		while (true) {
			if (avoiding && !inDanger) {
				startTime = System.currentTimeMillis();
				distance = poller.getReading();
				if (distance < FinalProject.THRESHOLD) {
					FinalProject.leftMotor.stop(true);
					FinalProject.rightMotor.stop(false);
					inDanger = true;
				}
				endTime = System.currentTimeMillis();
				if (endTime - startTime < SAMPLINGPERIOD) {
					try {
						Thread.sleep(SAMPLINGPERIOD - (endTime - startTime));
					} catch (InterruptedException e) {
					}
				}
			} else if (avoiding && inDanger) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				sensorMotor.off();
				sleepFor(0.5);
				while (FinalProject.usMotor.isMoving())
					continue;
				sleepFor(1);
				FinalProject.usMotor.rotateTo(sensorMotor.reference + 45);
				while (FinalProject.usMotor.isMoving())
					continue;
				int filter = 0;
				int measure = poller.getReading();
				FinalProject.leftMotor.setSpeed(200);
				FinalProject.rightMotor.setSpeed(200);
				do {

					startTime = System.currentTimeMillis();
					distance = poller.getReading();
					if (distance > FinalProject.THRESHOLD + 10 && filter < FILTERCONTROL) {
						filter++;
						// System.out.println("Filter =" + filter);
					} else if (distance > FinalProject.THRESHOLD + 10 && filter >= FILTERCONTROL) {
						FinalProject.leftMotor.stop(true);
						FinalProject.rightMotor.stop(false);
						// System.out.println("Avoided Object");
						sleepFor(1);
						FinalProject.usMotor.rotateTo(sensorMotor.reference);
						while (FinalProject.usMotor.isMoving())
							continue;
						sleepFor(1);
						sensorMotor.on = true;
						inDanger = false;
						break;
					} else {
						filter = 0;
						measure = distance;
					}
					if (measure > FinalProject.THRESHOLD && measure < FinalProject.THRESHOLD + 10) {
						FinalProject.leftMotor.setSpeed(250);
						FinalProject.rightMotor.setSpeed(250);
						FinalProject.leftMotor.forward();
						FinalProject.rightMotor.forward();
					} else if (measure < FinalProject.THRESHOLD) {
						FinalProject.leftMotor.setSpeed(250);
						FinalProject.rightMotor.setSpeed(125);
						FinalProject.leftMotor.forward();
						FinalProject.rightMotor.forward();
					} else {
						FinalProject.leftMotor.setSpeed(125);
						FinalProject.rightMotor.setSpeed(250);
						FinalProject.rightMotor.forward();
						FinalProject.leftMotor.forward();
					}
					endTime = System.currentTimeMillis();
					if (endTime - startTime < SAMPLINGPERIOD) {
						try {
							Thread.sleep(SAMPLINGPERIOD - (endTime - startTime));
						} catch (InterruptedException e) {
						}
					}
				} while (inDanger);

			} else {
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
				}
			}
		}
	}

	public void sleepFor(double x) {
		try {
			Thread.sleep((long) (x * 100));
		} catch (InterruptedException e) {
		}
	}

	public void on() {
		avoiding = true;
	}

	public void off() {
		avoiding = false;
	}
}
