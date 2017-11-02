package ca.mcgill.ecse211.dpmfinalprojectteam3;

public class Avoidance extends Thread {
	private Navigation gps;
	public boolean avoid;

	public Avoidance(Navigation gps) {
		this.gps = gps;

	}
}
