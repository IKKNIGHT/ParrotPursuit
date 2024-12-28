package org.firstinspires.ftc.teamcode.geometry;

public class WayPoint {
    Pose2d position;
    Transform2d tolerance;

    public WayPoint(Pose2d pos, Transform2d tolerance){
        this.position=pos;
        this.tolerance=tolerance;
    }
    public WayPoint(Pose2d pos, double tolerance){
        this(pos, new Transform2d(new Translation2d(tolerance, tolerance), Rotation2d.fromDegrees(tolerance)));
    }

    public Pose2d getPosition(){
        return position;
    }

    public Transform2d getTolerance() {
        return tolerance;
    }
}