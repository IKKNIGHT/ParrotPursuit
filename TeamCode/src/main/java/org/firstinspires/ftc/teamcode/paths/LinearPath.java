package org.firstinspires.ftc.teamcode.paths;

public class LinearPath extends Path {
    Point point1, point2;
    /**
     * Constructs a {@code LinearPath} object with the given start and end points.
     * @param p1 the start point of the path
     * @param p2 the end point of the path
     */
    public LinearPath(Point p1, Point p2){
        // Set the start and end points of the path
        point1 = p1;
        point2 = p2;
    }
    /**
     * gets the point on the path at a given time
     * @param t the time to get the point at
     * @return the point on the path at the given time
     */
    @Override
    public Point getPoint(double t) {
        return point1.times(1-t).add(point2.times(t));
    }
}
