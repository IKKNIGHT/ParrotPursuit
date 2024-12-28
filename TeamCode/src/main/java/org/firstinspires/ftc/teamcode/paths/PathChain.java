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
     * Constructs a {@code PathChain} object from a varargs array of paths.
     *
     * @param paths the paths to define the chain
     */
    public PathChain(Path... paths){
        this(List.of(paths));
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
     * @return the number of paths
     */
    public int getSegmentCount() {
        return segments.size();
    }

    /**
     * Inserts a new path at the specified index.`
     * @param index index you want to insert the path to
     * @param path the path to insert
     */
    public void insertPath(int index, Path path) {
        segments.add(index, path);
    }
    /**
     * Gets the path at the specified index.
     * @param index index of the path you want to get
     * @return the path at the specified index
     */
    public Path getPath(int index) {
        return segments.get(index);
    }

    /**
     * Deletes the path at the specified index.
     * @param index index of the path you want to delete
     */
    public void deletePath(int index) {
        segments.remove(index);
    }
}
