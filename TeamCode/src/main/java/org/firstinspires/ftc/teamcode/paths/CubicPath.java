package org.firstinspires.ftc.teamcode.paths;

public class CubicPath extends Path {
    //code for a cubic path
    Point point1, point2, point3, point4;

    /**
     * Constructs a {@code CubicPath} object with the given start and end points.
     * @param p1 First point of the path
     * @param p2 Second point of the path
     * @param p3 Third point of the path
     * @param p4 Fourth point of the path
     */
    public CubicPath(Point p1, Point p2, Point p3, Point p4){
        // Set the start and end points of the path
        point1 = p1;
        point2 = p2;
        point3 = p3;
        point4 = p4;
    }
    /**
     * gets the point on the path at a given time
     * @param t the time to get the point at
     * @return the point on the path at the given time
     */
    @Override
    public Point getPoint(double t) {
        return point1.times(Math.pow(1-t, 3)).add(point2.times(3*Math.pow(1-t, 2)*t)).add(point3.times(3*(1-t)*Math.pow(t, 2))).add(point4.times(Math.pow(t, 3)));
    }
}
