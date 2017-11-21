package ca.mcgill.ecse211.dpmfinalprojectteam3;

import java.util.LinkedList;

// TODO: Auto-generated Javadoc
// 
/**
 * Class used to construct the path of the robot right before starting
 * localization. While constructing the path, this class will take into account
 * the orientations of the search regions and how the river is constructed in
 * order to construct a path that will be viable for the robot to take. Have to
 * build the river differently depending on which parts of the river are
 * connected to red and green zones and which aren't. Also has to take into
 * account which zone is on which side to determine correct offset for robot
 * entering the green zone.
 * 
 * @author Sam Cleland
 * @since 11/19/17
 */
public class PathFinder {

	/** The navigation it is associated with to add the path to once complete. */
	Navigation gps;

	/** Linked list used as the main path for the robot once built and passed in. */
	LinkedList<Double> coordsList = new LinkedList<Double>();

	/**
	 * Linked list used for search region path, changes based on orientation of red
	 * or green and if searching in red or green zone.
	 */
	LinkedList<Integer> searchRegionList = new LinkedList<Integer>();

	/**
	 * Path finder needs a gps to find out where it needs to put the constructed
	 * pathes for search region into.
	 *
	 * @param gps
	 *            the navigation it will be passing the lists into
	 */
	public PathFinder(Navigation gps) {
		this.gps = gps;
	}

	/**
	 * Gets the path.
	 *
	 * @return the path
	 */
	public void getPath() {

		if (FinalProject.greenTeam == 3)
			addPathGreen();
		else
			addPathRed();
	}

	/**
	 * Adds the red path for the robot if the robot is on the red team.
	 */
	private void addPathRed() {
		// connected to the left horizontal part of the shallow water region
		if (isWithinRegion(FinalProject.SHLLX, FinalProject.SHLLY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			if (FinalProject.REDYONE > FinalProject.GREENYONE) {
				// have to travel to the left and then travel up the vertical portion of the
				// river
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.add((double) FinalProject.SHLLX - 0.5);
				coordsList.add((double) FinalProject.startingY);
				coordsList.add((double) FinalProject.SHLLX - 0.5);
				coordsList.add((double) middleY);
				coordsList.add((double) middleX);
				coordsList.add((double) middleY);
				coordsList.add((double) middleX);
				coordsList.add((double) FinalProject.SVLLY - 0.5);
				coordsList.add((double) FinalProject.URSRGX);
				coordsList.add((double) FinalProject.SVLLY - 0.5);
				coordsList.add((double) FinalProject.URSRGX);
				coordsList.add((double) FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.URSRGX);
				gps.setSearchRegionPath(searchRegionList);
				// Traversing the river from close to far, so have to change where we travel to
				// and go to the closer corner of the search region
			} else {
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.add((double) FinalProject.SHLLX - 0.5);
				coordsList.add((double) FinalProject.startingY);
				coordsList.add((double) FinalProject.SHLLX - 0.5);
				coordsList.add((double) middleY);
				coordsList.add((double) middleX);
				coordsList.add((double) middleY);
				coordsList.add((double) middleX);
				coordsList.add((double) FinalProject.SVURY + 0.5);
				coordsList.add((double) FinalProject.LLSRGX);
				coordsList.add((double) FinalProject.SVURY + 0.5);
				coordsList.add((double) FinalProject.LLSRGX);
				coordsList.add((double) FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				gps.setSearchRegionPath(searchRegionList);

			}
			// same as above except connected to the right part of the shallow water region
		} else if (isWithinRegion(FinalProject.SHURX, FinalProject.SHURY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			if (FinalProject.REDYONE > FinalProject.GREENYONE) {
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.add((double) FinalProject.SHURX + 0.5);
				coordsList.add((double) FinalProject.startingY);
				coordsList.add((double) FinalProject.SHURX + 0.5);
				coordsList.add((double) middleY);
				coordsList.add((double) middleX);
				coordsList.add((double) middleY);
				coordsList.add((double) middleX);
				coordsList.add((double) FinalProject.SVLLY - 0.5);
				coordsList.add((double) FinalProject.URSRGX);
				coordsList.add((double) FinalProject.SVLLY - 0.5);
				coordsList.add((double) FinalProject.URSRGX);
				coordsList.add((double) FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.URSRGX);
				gps.setSearchRegionPath(searchRegionList);

			} else {
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.add((double) FinalProject.SHURX + 0.5);
				coordsList.add((double) FinalProject.startingY);
				coordsList.add((double) FinalProject.SHURX + 0.5);
				coordsList.add((double) middleY);
				coordsList.add((double) middleX);
				coordsList.add((double) middleY);
				coordsList.add((double) middleX);
				coordsList.add((double) FinalProject.SVURY + 0.5);
				coordsList.add((double) FinalProject.LLSRGX);
				coordsList.add((double) FinalProject.SVURY + 0.5);
				coordsList.add((double) FinalProject.LLSRGX);
				coordsList.add((double) FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				gps.setSearchRegionPath(searchRegionList);

			}
			// connected to the vertical component of the river, then have to check what is
			// connected to the green zone, left or right part of the shallow water region
		} else if (isWithinRegion(FinalProject.SVLLX, FinalProject.SVLLY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			if (isWithinRegion(FinalProject.SHLLX, FinalProject.SHLLY, FinalProject.GREENXONE, FinalProject.GREENYONE,
					FinalProject.GREENXTWO, FinalProject.GREENYTWO)) {
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.startingY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) FinalProject.LLSRGY);
				coordsList.addLast((double) FinalProject.LLSRGX);
				coordsList.addLast((double) FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				gps.setSearchRegionPath(searchRegionList);

			} else {
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.startingY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHURX + 0.5);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHURX + 0.5);
				coordsList.addLast((double) FinalProject.LLSRGY);
				coordsList.addLast((double) FinalProject.LLSRGX);
				coordsList.addLast((double) FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				gps.setSearchRegionPath(searchRegionList);

			}
			// same as above
		} else if (isWithinRegion(FinalProject.SVURX, FinalProject.SVURY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			if (isWithinRegion(FinalProject.SHLLX, FinalProject.SHLLY, FinalProject.GREENXONE, FinalProject.GREENYONE,
					FinalProject.GREENXTWO, FinalProject.GREENYTWO)) {
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.startingY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) FinalProject.URSRGY);
				coordsList.addLast((double) FinalProject.URSRGX);
				coordsList.addLast((double) FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.URSRGX);
				gps.setSearchRegionPath(searchRegionList);

			} else {
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.startingY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHURX + 0.5);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHURX + 0.5);
				coordsList.addLast((double) FinalProject.URSRGY);
				coordsList.addLast((double) FinalProject.URSRGX);
				coordsList.addLast((double) FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.URSRGY);
				searchRegionList.add(FinalProject.URSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.LLSRGY);
				searchRegionList.add(FinalProject.LLSRGX);
				searchRegionList.add(FinalProject.URSRGX);
				gps.setSearchRegionPath(searchRegionList);
			}
		}
		// should always be the same
		coordsList.addLast((double) FinalProject.zipgreenXc);
		coordsList.addLast((double) FinalProject.URSRGY);
		coordsList.addLast((double) FinalProject.zipgreenXc);
		coordsList.addLast((double) FinalProject.zipgreenYc);
		coordsList.addLast((double) FinalProject.startingX);
		coordsList.addLast((double) FinalProject.zipredYc);
		coordsList.addLast((double) FinalProject.startingX);
		coordsList.addLast((double) FinalProject.startingY);
		gps.setPath(coordsList);
	}

	/**
	 * Adds the path green for the robot if the robot is on the green team.
	 */
	private void addPathGreen() {
		System.out.println("ADDING GREEN PATH");
		// Don't want to run into search region so we go around it
		if (FinalProject.startingY >= FinalProject.LLSRGY && FinalProject.startingY <= FinalProject.URSRGY) {
			coordsList.add((double) FinalProject.startingX);
			coordsList.add((double) FinalProject.URSRGY);
			coordsList.add((double) FinalProject.zipgreenXc);
			coordsList.add((double) FinalProject.URSRGY);
			coordsList.add((double) FinalProject.zipgreenXc);
			coordsList.add((double) FinalProject.zipgreenYc);
			// same as above for if green zone is on the far side
		} else if (FinalProject.startingY <= FinalProject.URSRGY && FinalProject.startingY >= FinalProject.LLSRGY) {
			coordsList.add((double) FinalProject.startingX);
			coordsList.add((double) FinalProject.LLSRGY);
			coordsList.add((double) FinalProject.zipgreenXc);
			coordsList.add((double) FinalProject.LLSRGY);
			coordsList.add((double) FinalProject.zipgreenXc);
			coordsList.add((double) FinalProject.zipgreenYc);
			// else is ok to go the normal way
		} else {
			coordsList.add((double) FinalProject.startingX);
			coordsList.add((double) FinalProject.zipgreenYc);
			coordsList.add((double) FinalProject.zipgreenXc);
			coordsList.add((double) FinalProject.zipgreenYc);
		}
		// change path based on which zone is on which side, such as to go to LL or UR
		// search region
		if (FinalProject.GREENYONE < FinalProject.REDYONE) {
			System.out.println("RED ZONE FURTHER AWAY THAN GREEN");
			coordsList.add((double) FinalProject.LLSRRX);
			coordsList.add((double) FinalProject.zipredYc);
			coordsList.add((double) FinalProject.LLSRRX);
			coordsList.add((double) FinalProject.LLSRRY);
			searchRegionList.addLast(FinalProject.LLSRRX);
			searchRegionList.addLast(FinalProject.LLSRRY);
			searchRegionList.addLast(FinalProject.LLSRRX);
			searchRegionList.addLast(FinalProject.URSRRY);
			searchRegionList.addLast(FinalProject.URSRRX);
			searchRegionList.addLast(FinalProject.URSRRY);
			searchRegionList.addLast(FinalProject.URSRRX);
			searchRegionList.addLast(FinalProject.LLSRRY);
			gps.setSearchRegionPath(searchRegionList);
		} else {
			coordsList.add((double) FinalProject.zipredXc);
			coordsList.add((double) FinalProject.URSRRY);
			coordsList.add((double) FinalProject.URSRRX);
			coordsList.add((double) FinalProject.URSRRY);
			searchRegionList.addLast(FinalProject.URSRRX);
			searchRegionList.addLast(FinalProject.URSRRY);
			searchRegionList.addLast(FinalProject.URSRRX);
			searchRegionList.addLast(FinalProject.LLSRRY);
			searchRegionList.addLast(FinalProject.LLSRRX);
			searchRegionList.addLast(FinalProject.LLSRRY);
			searchRegionList.addLast(FinalProject.LLSRRX);
			searchRegionList.addLast(FinalProject.URSRRY);
			gps.setSearchRegionPath(searchRegionList);
		}
		// check if the lower left corner of the shallow region is connected to the red
		// region
		if (isWithinRegion(FinalProject.SHLLX, FinalProject.SHLLY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			System.out.println("ADDING LOWER LEFT HORIZONTAL");
			if (FinalProject.GREENYONE < FinalProject.REDYONE) {
				// go to the average of these coordinates, this allows it to scale with a larger
				// river region
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) FinalProject.URSRRY);
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.SVLLY - 0.5);
				coordsList.addLast((double) FinalProject.startingX);
				coordsList.addLast((double) FinalProject.SVLLY - 0.5);
				coordsList.addLast((double) FinalProject.startingX);
				coordsList.addLast((double) FinalProject.startingY);
				gps.setPath(coordsList);
			} else {
				// change path alittle bit if traversing from close to far side, have to travel
				// +0.5 instead of -0.5 and to search vertical UR instead of LL
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) FinalProject.URSRRY);
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.SVURY + 0.5);
				coordsList.addLast((double) FinalProject.startingX);
				coordsList.addLast((double) FinalProject.SVURY + 0.5);
				coordsList.addLast((double) FinalProject.startingX);
				coordsList.addLast((double) FinalProject.startingY);
				gps.setPath(coordsList);
			} // same as above case except for if it is the right side of the shallow water
				// region
		} else if (isWithinRegion(FinalProject.SHURX, FinalProject.SHURY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			System.out.println("ADDING UPPER RIGHT HORIZONTAL");
			if (FinalProject.GREENYONE < FinalProject.REDYONE) {
				double middleX = (FinalProject.SVURX + FinalProject.SVLLX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.addLast((double) FinalProject.SHURX + 0.5);
				coordsList.addLast((double) FinalProject.URSRRY);
				coordsList.addLast((double) FinalProject.SHURX + 0.5);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.SVLLY - 0.5);
				coordsList.addLast((double) FinalProject.startingX);
				coordsList.addLast((double) FinalProject.SVLLY - 0.5);
				coordsList.addLast((double) FinalProject.startingX);
				coordsList.addLast((double) FinalProject.startingY);
				gps.setPath(coordsList);
			} else {
				double middleX = (FinalProject.SVURX + FinalProject.SVLLX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.addLast((double) FinalProject.SHURX + 0.5);
				coordsList.addLast((double) FinalProject.LLSRRY);
				coordsList.addLast((double) FinalProject.SHURX + 0.5);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.SVURY + 0.5);
				coordsList.addLast((double) FinalProject.startingX);
				coordsList.addLast((double) FinalProject.SVURY + 0.5);
				coordsList.addLast((double) FinalProject.startingX);
				coordsList.addLast((double) FinalProject.startingY);
				gps.setPath(coordsList);
			} // this part executes if the vertical part (from close to far) is connected to
				// the red region, have
				// to traverse slightly different depending on what the green region is
				// connected to
		} else if (isWithinRegion(FinalProject.SVURX, FinalProject.SVURY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)
				|| isWithinRegion(FinalProject.SVLLX, FinalProject.SVLLY, FinalProject.REDXONE, FinalProject.REDYONE,
						FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			// means that the LL region is what connects the shallow water region on the
			// other side, so we have to travel to it in the end
			System.out.println("ADDING VERTICAL COMPONENT");

			if (isWithinRegion(FinalProject.SHLLX, FinalProject.SHLLY, FinalProject.GREENXONE, FinalProject.GREENYONE,
					FinalProject.GREENXTWO, FinalProject.GREENYTWO)) {
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.URSRRY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) FinalProject.startingY);
				coordsList.addLast((double) FinalProject.startingX);
				coordsList.addLast((double) FinalProject.startingY);
				gps.setPath(coordsList);
				// Same idea as above for the upper right
			} else if (isWithinRegion(FinalProject.SHURX, FinalProject.SHURY, FinalProject.GREENXONE,
					FinalProject.GREENYONE, FinalProject.GREENXTWO, FinalProject.GREENYTWO)) {
				double middleX = (FinalProject.SVLLX + FinalProject.SVURX) / 2.0;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2.0;
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.URSRRY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHURX + 0.5);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHURX + 0.5);
				coordsList.addLast((double) FinalProject.startingY);
				coordsList.addLast((double) FinalProject.startingX);
				coordsList.addLast((double) FinalProject.startingY);
				gps.setPath(coordsList);
			}
			// same case as above except it is red being the far zone, so have to change
			// where the robot is coming from
		}
		gps.setPath(coordsList);
	}

	/**
	 * Checks if the x and y passed in are within xf, yf and xs, ys. Useful for
	 * checking if a river coordinate is connected to a zone. In this case xf<=xs,
	 * yf<=ys
	 *
	 * @param x
	 *            the x
	 * @param y
	 *            the y
	 * @param xf
	 *            the xf
	 * @param yf
	 *            the yf
	 * @param xs
	 *            the xs
	 * @param ys
	 *            the ys
	 * @return true, if x and y are in the region created by f and s coordinates or
	 *         x is in between xf and xs and y is in between yf and ys.
	 */
	public static boolean isWithinRegion(double x, double y, double xf, double yf, double xs, double ys) {
		return (x >= xf && x <= xs) && (y >= yf && y <= ys);
	}
}
