package org.firstinspires.ftc.teamcode.paths;

import java.util.ArrayList;
import java.util.List;

public class SplinePath extends Path {
    private final List<CubicPath> cubicPaths = new ArrayList<>();

    /**
     * Constructs a {@code SplinePath} object from a list of control points.
     * Each segment of the spline is represented by a cubic interpolation between control points.
     * <br><br>
     * <strong>Control points must be at least 4 and a multiple of 3 plus 1</strong>
     *
     * @param controlPoints the list of points to define the spline
     */
    public SplinePath(List<Point> controlPoints) {
        if (!isValidPath(controlPoints)) {
            throw new IllegalArgumentException("Control points must be at least 4 and a multiple of 3 plus 1.");
        }

        for (int i = 0; i <= controlPoints.size() - 4; i += 3) {
            Point p1 = controlPoints.get(i);
            Point p2 = controlPoints.get(i + 1);
            Point p3 = controlPoints.get(i + 2);
            Point p4 = controlPoints.get(i + 3);
            cubicPaths.add(new CubicPath(p1, p2, p3, p4));
        }
    }

    public SplinePath(Point... controlPoints) {
        this(List.of(controlPoints));
    }

    /**
     * Gets the point on the spline at a given time.
     *
     * @param t the time (0 to 1) to get the point at
     * @return the point on the spline at the given time
     */
    @Override
    public Point getPoint(double t) {
        if (t <= 0) return cubicPaths.get(0).getPoint(0);
        if (t >= 1) return cubicPaths.get(cubicPaths.size() - 1).getPoint(1);

        double segmentDuration = 1.0 / cubicPaths.size();
        int segmentIndex = (int) (t / segmentDuration);
        double segmentT = (t % segmentDuration) / segmentDuration;

        return cubicPaths.get(segmentIndex).getPoint(segmentT);
    }

    /**
     * checks if the path is valid
     * @param controlPoints List of control points that you would like to check
     * @return true or false based off of the control points being valid.
     */
    public boolean isValidPath(List<Point> controlPoints ){
        return controlPoints.size() >= 4 && (controlPoints.size() - 1) % 3 == 0;
    }
}
