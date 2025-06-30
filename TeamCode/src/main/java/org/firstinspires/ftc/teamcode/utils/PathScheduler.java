package org.firstinspires.ftc.teamcode.utils;


import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.UpdateAction;
import org.firstinspires.ftc.teamcode.paths.Path;

import java.util.List;

/**
 * Path Scheduler class that schedules paths for robots to follow in sequential order, to make a multi path following robot paradigm.
 */
public class PathScheduler {
    private List<Path> paths;
    MecanumDrive drive;

    /**
     * Constructor for the PathScheduler class.
     * @param paths List of paths to be scheduled.
     * @param drive MecanumDrive instance associated with the paths.
     */
    public PathScheduler(List<Path> paths, MecanumDrive drive) {
        if (paths == null) {
            throw new IllegalArgumentException("paths cannot be null");
        }
        if (drive == null) {
            throw new IllegalArgumentException("drive cannot be null");
        }
        if (paths.isEmpty()) {
            throw new IllegalArgumentException("paths cannot be empty");
        }

        this.paths = paths;
        this.drive = drive;
    }

    /**
     * Runs scheduled paths in sequential order.
     * @param opModeIsActive Flag indicating whether the OpMode is active. Pass opModeIsActive() from the OpMode.
     */
    public void runScheduledPaths(boolean opModeIsActive) {
        for (Path path : paths) {
            drive.followPath(path);
            while (drive.isFollowingPath() && opModeIsActive) {
                drive.updateLocalizer();
                drive.updatePurePursuit();

            }
            paths.remove(0);
        }
    }

    /**
     * Runs scheduled paths in sequential order with an update action.
     * @see UpdateAction
     * @param opModeIsActive Flag indicating whether the OpMode is active. Pass opModeIsActive() from the OpMode.
     * @param action Update action to be run during path execution.
     */
    public void runScheduledPaths(boolean opModeIsActive, UpdateAction action) {
        for (Path path : paths) {
            drive.followPath(path);
            while (drive.isFollowingPath() && opModeIsActive) {
                drive.updateLocalizer();
                drive.updatePurePursuit();
                action.onUpdate(); // run update action
            }
            paths.remove(0);
        }
    }

    /**
     * Gets the list of paths.
     * @return List of paths.
     */
    public List getPaths(){
        return paths;
    }

    /**
     * Sets the list of paths.
     * @param paths List of paths to be set.
     */
    public void setPaths(List<Path> paths){
        if (paths == null) {
            throw new IllegalArgumentException("paths cannot be null");
        }
        if (paths.isEmpty()) {
            throw new IllegalArgumentException("paths cannot be empty");
        }
        this.paths = paths;
    }

    /**
     * Schedules a path for execution.
     * @param path Path to be scheduled.
     */
    public void schedulePath(Path path){
        paths.add(path);
    }

    /**
     * Gets the current path.
     * @return Current path.
     */
    public Path getCurrentPath(){
        return paths.get(0);
    }

    /**
     * Removes a path from the list.
     * @param path Path to be removed.
     */
    public void removePath(Path path){
        paths.remove(path);
    }

    /**
     * Kills the scheduler and stops the path following.
     */
    public void killScheduler(){
        paths.clear();
        drive.stopPathFollowing();
    }

}
