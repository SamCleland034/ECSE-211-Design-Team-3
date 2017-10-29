package ca.mcgill.ecse211.dpmfinalprojectteam3;

// TODO: Auto-generated Javadoc
/**
 * Implements two methods used for reading ultrasonic data and determines
 * whether to react to the data or not depending on the values (Too close or too
 * far from a wall).
 */
public interface UltrasonicController {

	/**
	 * Processes data read from the US sensor with filtering included. Determines
	 * if the distance is correct, too close or too far. Robot responds accordingly to
	 * each of the 3 cases.
	 *
	 * @param distance,
	 *            distance value read through the poller.
	 */
	public void processUSData(int distance);

	/**
	 * Reads value from the sensor and updates it into the controller
	 *
	 * @return the distance read
	 */
	public int readUSDistance();
}
