package org.firstinspires.ftc.teamcode.geometry;

public class WayPoint {
    Pose2d position;
    Transform2d tolerance;

    /**
     * Constructs a {@code WayPoint} object with a given position and transform2d tolerance.
     * @param pos the position of the waypoint
     * @param tolerance the tolerance of the waypoint
     */
    public WayPoint(Pose2d pos, Transform2d tolerance){
        this.position=pos;
        this.tolerance=tolerance;
    }
    /**
     * Constructs a {@code WayPoint} object with a given position and double tolerance.
     * @param pos the position of the waypoint
     */
    public WayPoint(Pose2d pos, double tolerance){
        this(pos, new Transform2d(new Translation2d(tolerance, tolerance), Rotation2d.fromDegrees(tolerance)));
    }

    /**
     * Gets the position of the waypoint.
     * @return the position of the waypoint
     */
    public Pose2d getPosition(){
        return position;
    }

    /**
     * Gets the tolerance of the waypoint.
     * @return the tolerance of the waypoint
     */
    public Transform2d getTolerance() {
        return tolerance;
    }
}