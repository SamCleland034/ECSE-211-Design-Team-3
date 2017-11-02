package ca.mcgill.ecse211.dpmfinalprojectteam3;

public class Avoidance extends Thread {
	private Navigation gps;
	private static final long SAMPLINGPERIOD = 50;
	private UltrasonicPoller poller;
	private static final int FILTERCONTROL = 40;
	public boolean avoiding;
	public boolean inDanger;
	private SensorRotation sensorMotor;

	public Avoidance(Navigation gps) {
		this.gps = gps;

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
				sensorMotor.on = false;
				int filter = 0;
				FinalProject.usMotor.rotate(-45);
				FinalProject.leftMotor.setSpeed(200);
				FinalProject.rightMotor.setSpeed(200);
				while (inDanger) {
					startTime = System.currentTimeMillis();
					distance = poller.getReading();
					if (distance > 255 && filter < FILTERCONTROL) {
						filter++;
					} else if (distance > 255 && filter > FILTERCONTROL) {
						FinalProject.leftMotor.stop(true);
						FinalProject.rightMotor.stop(false);
						inDanger = false;
						sensorMotor.on = true;
						break;
					} else {
						filter = 0;
					}
					if (distance > FinalProject.THRESHOLD - 5 && distance < FinalProject.THRESHOLD - 5) {
						FinalProject.leftMotor.setSpeed(200);
						FinalProject.rightMotor.setSpeed(200);
						FinalProject.leftMotor.forward();
						FinalProject.rightMotor.forward();
					} else if (distance < FinalProject.THRESHOLD - 5) {
						FinalProject.leftMotor.setSpeed(200);
						FinalProject.rightMotor.setSpeed(100);
						FinalProject.leftMotor.forward();
						FinalProject.rightMotor.forward();
					} else {
						FinalProject.leftMotor.setSpeed(100);
						FinalProject.rightMotor.setSpeed(200);
						FinalProject.rightMotor.forward();
						FinalProject.leftMotor.forward();
					}
					endTime = System.currentTimeMillis();
					if (endTime - startTime < 2 * SAMPLINGPERIOD) {
						try {
							Thread.sleep(2 * SAMPLINGPERIOD - (endTime - startTime));
						} catch (InterruptedException e) {
						}
					}
				}

			} else {
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
				}
			}
		}
	}
}
