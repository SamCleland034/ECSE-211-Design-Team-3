package ca.mcgill.ecse211.dpmfinalprojectteam3;

/**
 * Enumeration used to keep track of what state the robot is in. The first
 * states, STARTINGLOCALIZATION , WIFI AND IDLE, are not that important to model
 * but included them to show the flow. However, ZIPLOCALIZATION, NAVIGATION,
 * FLAGSEARCH determine what is going on with the robot. If the robot is in the
 * navigating state, it will cycle through the coordinates that it was passed in
 * the beginning before transitioning to this state. To transition to the
 * flagsearch or ziplocalization state, is through the @seestartNav() method in
 * the Navigation class. In short terms, the robot will switch states based on
 * the coordinates that the robot just travelled to. So if the robot travelled
 * to the lower left of the red search region, it will switch into the
 * flagsearch state.
 */
public enum Stage {

	STARTINGLOCALIZATION,

	NAVIGATION,

	ZIPLOCALIZATION,

	FLAGSEARCH,

	WIFI,

	IDLE,

	FINISHED
}
