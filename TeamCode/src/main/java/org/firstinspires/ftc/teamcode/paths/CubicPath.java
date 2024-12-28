package org.firstinspires.ftc.teamcode.paths;

public class CubicPath extends Path {
    //code for a cubic path
    Point point1, point2, point3, point4;
    public CubicPath(Point p1, Point p2, Point p3, Point p4){
        // Set the start and end points of the path
        point1 = p1;
        point2 = p2;
        point3 = p3;
        point4 = p4;
    }
    @Override
    public Point getPoint(double t) {
        return point1.times(Math.pow(1-t, 3)).add(point2.times(3*Math.pow(1-t, 2)*t)).add(point3.times(3*(1-t)*Math.pow(t, 2))).add(point4.times(Math.pow(t, 3)));
    }
}
