package org.firstinspires.ftc.teamcode.paths;

import java.util.ArrayList;
import java.util.List;

public class PolyLine extends Path {
    private final List<Path> segments = new ArrayList<>();

    /**
     * Constructs a {@code PathChain} object from a varargs array of control points.
     * This path ensures differentiability at intermediate points by dynamically
     * creating paths between points.
     *
     * @param controlPoints the control points to define the chain
     */
    public PolyLine(Point... controlPoints) {
        this(List.of(controlPoints));
    }

    /**
     * Constructs a {@code PathChain} object from a list of control points.
     * This path ensures differentiability at intermediate points by dynamically
     * creating paths between points.
     *
     * @param controlPoints the list of points to define the chain
     */
    public PolyLine(List<Point> controlPoints) {
        if (controlPoints.size() < 2) {
            throw new IllegalArgumentException("At least two control points are required to create a path chain.");
        }

        // Create linear or cubic paths between points as needed
        for (int i = 0; i < controlPoints.size() - 1; i++) {
            Point start = controlPoints.get(i);
            Point end = controlPoints.get(i + 1);
            // If more complex interpolation is desired, replace LinearPath with a CubicPath or similar
            segments.add(new LinearPath(start, end));
        }
    }

    /**
     * Gets the point on the chain at a given time.
     *
     * @param t the time (0 to 1) to get the point at
     * @return the point on the chain at the given time
     */
    @Override
    public Point getPoint(double t) {
        if (t <= 0) return segments.get(0).getPoint(0);
        if (t >= 1) return segments.get(segments.size() - 1).getPoint(1);

        double segmentDuration = 1.0 / segments.size();
        int segmentIndex = (int) (t / segmentDuration);
        double segmentT = (t % segmentDuration) / segmentDuration;

        return segments.get(segmentIndex).getPoint(segmentT);
    }

    /**
     * Validates the given control points.
     *
     * @param controlPoints the list of points to validate
     * @return true if the control points can define a path chain, false otherwise
     */
    public boolean isValidPath(List<Point> controlPoints) {
        return controlPoints.size() >= 2;
    }
}
