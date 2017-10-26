package ca.mcgill.ecse211.dpmfinalprojectteam3;

// TODO: Auto-generated Javadoc
/**
 * The Enum LocalizationType, created enum to allow easy construction when
 * creating the constructor for ultrasonic localization
 */
public enum LocalizationType {

	/**
	 * The fallingedge. Going from seeing a high distance to a low distance while
	 * performing ultrasonic localization
	 */
	// enumeration to distinguish which localization type to use for constructor
	FALLINGEDGE,
	/**
	 * The risingedge. Going from seeing a low distance to a high distance while
	 * performing ultrasonic localization
	 */
	RISINGEDGE
}
