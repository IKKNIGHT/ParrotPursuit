package org.firstinspires.ftc.teamcode.paths;

import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;

public abstract class Path {
    private double t = 0;
    private Rotation2d endHeading;

    /**
     * resets the time of the path
     */
    public void reset(){
        t=0;
    }
    /**
     * increments the time of the path
     */
    public void incrementPath(){
        if(t < 1){
            t += 0.01;
        }else{
            t = 1;
        }
    }
    /**
     * gets the point on the path at a given time
     * @param t the time to get the point at
     * @return the point on the path at the given time
     */
    public abstract Point getPoint(double t);
    /**
     * gets the end pose of the path
     * @return the end pose of the path
     */
    public Pose2d getEndPose(){
        return new Pose2d(getPoint(1).getX(), getPoint(1).getY(), endHeading);
    }

    /**
     * gets the current time of the path
     * @return the current time of the path
     */
    public double getCurrTime(){
        return t;
    }
}
