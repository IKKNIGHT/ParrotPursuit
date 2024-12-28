package org.firstinspires.ftc.teamcode.paths;

public class LinearPath extends Path {
    Point point1, point2;
    public LinearPath(Point p1, Point p2){
        // Set the start and end points of the path
        point1 = p1;
        point2 = p2;
    }
    @Override
    public Point getPoint(double t) {
        return point1.times(1-t).add(point2.times(t));
    }
}
