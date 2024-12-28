package org.firstinspires.ftc.teamcode.paths;

import java.util.ArrayList;
import java.util.List;

public class PathChain extends Path {
    private final List<Path> segments = new ArrayList<>();

    /**
     * Constructs a {@code PathChain} object from a list of paths.
     *
     * @param paths the list of paths to define the chain
     */
    public PathChain(List<Path> paths) {
        if (paths.isEmpty()) {
            throw new IllegalArgumentException("At least one path is required to create a path chain.");
        }
        segments.addAll(paths);
    }

    /**
     * Adds a new path to the chain.
     *
     * @param path the path to add to the chain
     */
    public void addPath(Path path) {
        segments.add(path);
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
     * Gets the number of paths in the chain.
     *
     * @return the number of paths
     */
    public int getSegmentCount() {
        return segments.size();
    }
}
