package org.firstinspires.ftc.teamcode.paths;

import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;

public abstract class Path {
    private double t = 0;
    private Rotation2d endHeading;
    public void reset(){
        t=0;
    }
    public void incrementPath(){
        if(t < 1){
            t += 0.01;
        }else{
            t = 1;
        }
    }
    public abstract Point getPoint(double t);
    public Pose2d getEndPose(){
        return new Pose2d(getPoint(1).getX(), getPoint(1).getY(), endHeading);
    }
    public double getCurrTime(){
        return t;
    }
}
