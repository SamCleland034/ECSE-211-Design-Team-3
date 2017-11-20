package ca.mcgill.ecse211.dpmfinalprojectteam3;

import java.util.LinkedList;

// 
/**
 * Thread used to construct the path of the robot while the robot is localizing.
 * While constructing the path, this class will take into account the
 * orientations of the search regions and how the river is constructed in order
 * to construct a path that will be viable for the robot to take.
 */
public class PathFinder extends Thread {

	/** The navigation it is associated with to add the path to once complete. */
	Navigation gps;

	/** Linked list used as the main path for the robot once built and passed in */
	LinkedList<Double> coordsList = new LinkedList<Double>();

	/**
	 * Linked list used for search region path, changes based on orientation of red
	 * or green and if searching in red or green zone
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

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#start()
	 */
	public void start() {

		if (FinalProject.greenTeam == 3)
			addPathGreen();
		else
			addPathRed();
	}

	/**
	 * Adds the red path for the robot if the robot is on the red team.
	 */
	private void addPathRed() {
		if (isWithinRegion(FinalProject.SHLLX, FinalProject.SHLLY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			if (FinalProject.REDYONE > FinalProject.GREENYONE) {
				double middleX = (FinalProject.SHLLX + FinalProject.SVURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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

			} else {
				double middleX = (FinalProject.SHLLX + FinalProject.SVURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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
		} else if (isWithinRegion(FinalProject.SHURX, FinalProject.SHURY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			if (FinalProject.REDYONE > FinalProject.GREENYONE) {
				double middleX = (FinalProject.SHLLX + FinalProject.SVURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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
				double middleX = (FinalProject.SHLLX + FinalProject.SVURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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
		} else if (isWithinRegion(FinalProject.SVLLX, FinalProject.SVLLY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			if (isWithinRegion(FinalProject.SHLLX, FinalProject.SHLLY, FinalProject.GREENXONE, FinalProject.GREENYONE,
					FinalProject.GREENXTWO, FinalProject.GREENYTWO)) {
				double middleX = (FinalProject.SVLLX + FinalProject.SHURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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
				double middleX = (FinalProject.SVLLX + FinalProject.SHURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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
		} else if (isWithinRegion(FinalProject.SVURX, FinalProject.SVURY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			if (isWithinRegion(FinalProject.SHLLX, FinalProject.SHLLY, FinalProject.GREENXONE, FinalProject.GREENYONE,
					FinalProject.GREENXTWO, FinalProject.GREENYTWO)) {
				double middleX = (FinalProject.SVLLX + FinalProject.SHURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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
				double middleX = (FinalProject.SVLLX + FinalProject.SHURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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
		if (FinalProject.startingY == FinalProject.LLSRGY) {
			coordsList.add((double) FinalProject.startingX);
			coordsList.add((double) FinalProject.URSRGY);
			coordsList.add((double) FinalProject.zipgreenXc);
			coordsList.add((double) FinalProject.URSRGY);
			coordsList.add((double) FinalProject.zipgreenXc);
			coordsList.add((double) FinalProject.zipgreenYc);

		} else if (FinalProject.startingY == FinalProject.URSRGY) {
			coordsList.add((double) FinalProject.startingX);
			coordsList.add((double) FinalProject.LLSRGY);
			coordsList.add((double) FinalProject.zipgreenXc);
			coordsList.add((double) FinalProject.LLSRGY);
			coordsList.add((double) FinalProject.zipgreenXc);
			coordsList.add((double) FinalProject.zipgreenYc);
		} else {
			coordsList.add((double) FinalProject.startingX);
			coordsList.add((double) FinalProject.zipgreenYc);
			coordsList.add((double) FinalProject.zipgreenXc);
			coordsList.add((double) FinalProject.zipgreenYc);
		}

		if (FinalProject.GREENYONE < FinalProject.REDYONE) {
			coordsList.add((double) FinalProject.zipredXc);
			coordsList.add((double) FinalProject.LLSRRY);
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
			if (FinalProject.GREENYONE < FinalProject.REDYONE) {
				double middleX = (FinalProject.SVLLX + FinalProject.SHURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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
				double middleX = (FinalProject.SVLLX + FinalProject.SHURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) FinalProject.LLSRRY);
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
			}
		} else if (isWithinRegion(FinalProject.SHURX, FinalProject.SHURY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			if (FinalProject.GREENYONE < FinalProject.REDYONE) {
				double middleX = (FinalProject.SVURX + FinalProject.SHLLX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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
				double middleX = (FinalProject.SVURX + FinalProject.SHLLX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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
			}
		} else if (isWithinRegion(FinalProject.SVURX, FinalProject.SVURY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			if (isWithinRegion(FinalProject.SHLLX, FinalProject.SHLLY, FinalProject.GREENXONE, FinalProject.GREENYONE,
					FinalProject.GREENXTWO, FinalProject.GREENYTWO)) {
				double middleX = (FinalProject.SVLLX + FinalProject.SHURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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
			} else if (isWithinRegion(FinalProject.SHURX, FinalProject.SHURY, FinalProject.GREENXONE,
					FinalProject.GREENYONE, FinalProject.GREENXTWO, FinalProject.GREENYTWO)) {
				double middleX = (FinalProject.SVLLX + FinalProject.SHURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
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

		} else if (isWithinRegion(FinalProject.SVLLX, FinalProject.SVLLY, FinalProject.REDXONE, FinalProject.REDYONE,
				FinalProject.REDXTWO, FinalProject.REDYTWO)) {
			if (isWithinRegion(FinalProject.SHLLX, FinalProject.SHLLY, FinalProject.GREENXONE, FinalProject.GREENYONE,
					FinalProject.GREENXTWO, FinalProject.GREENYTWO)) {
				double middleX = (FinalProject.SVLLX + FinalProject.SHURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.LLSRRY);
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) middleY);
				coordsList.addLast((double) FinalProject.SHLLX - 0.5);
				coordsList.addLast((double) FinalProject.startingY);
				coordsList.addLast((double) FinalProject.startingX);
				coordsList.addLast((double) FinalProject.startingY);

				gps.setPath(coordsList);
			} else if (isWithinRegion(FinalProject.SHURX, FinalProject.SHURY, FinalProject.GREENXONE,
					FinalProject.GREENYONE, FinalProject.GREENXTWO, FinalProject.GREENYTWO)) {
				double middleX = (FinalProject.SVLLX + FinalProject.SHURX) / 2;
				double middleY = (FinalProject.SHLLY + FinalProject.SHURY) / 2;
				coordsList.addLast((double) middleX);
				coordsList.addLast((double) FinalProject.LLSRRY);
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

		}

	}

	/**
	 * Checks if the x and y passed in are within xf, yf and xs, ys. Useful for
	 * checking if a river coordinate is connected to a zone.
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
	 * @return true, if x and y are in the region created by f and s coordinates
	 */
	public static boolean isWithinRegion(double x, double y, double xf, double yf, double xs, double ys) {
		return (x >= xf && x <= xs) && (y >= yf && y <= ys);
	}
}
