package org.firstinspires.ftc.teamcode.paths;

import androidx.annotation.NonNull;

public class Point {
    private double x, y;

    /**
     * Constructs a {@code Point} object with the given x and y coordinates.
     * @param x 'X' coord of the point
     * @param y 'Y' coord of the point
     */
    public Point(double x, double y){
        this.x=x;
        this.y=y;
    }

    /**
     * Gets the 'X' coordinate of the point.
     * @return the 'X' coordinate of the point
     */
    public double getX() {
        return x;
    }
    /**
     * Gets the 'Y' coordinate of the point.
     * @return the 'Y' coordinate of the point
     */
    public double getY() {
        return y;
    }

    /**
     * multiplies the point by a scalar
     * @param amount the scalar
     * @return the new point after the scalar is multiplied
     */
    public Point times(double amount){
        return new Point(x*amount, y*amount);
    }
    /**
     * adds two points together
     * @param other the other point
     * @return the new point after the two points are added
     */
    public Point add(Point other){
        return new Point(this.x+other.getX(), this.y+other.getY());
    }
    /**
     * subtracts two points
     * @param other the other point
     * @return the new point after the two points are subtracted
     */
    public Point subtract(Point other){
        return new Point(this.x-other.getX(), this.y-other.getY());
    }
    /**
     * finds the distance between two points
     * @param other the other point
     * @return the distance between the two points
     */
    public double distance(Point other){
        return Math.sqrt(Math.pow(this.x-other.getX(), 2)+Math.pow(this.y-other.getY(), 2));
    }
    /**
     * finds the dot product of two points
     * @param other the other point
     * @return the dot product of the two points
     */
    public double dot(Point other){
        return this.x*other.getX()+this.y*other.getY();
    }
    /**
     * finds the cross product of two points
     * @param other the other point
     * @return the cross product of the two points
     */
    public double cross(Point other){
        return this.x*other.getY()-this.y*other.getX();
    }
    /**
     * finds the magnitude of the point
     * @return the magnitude of the point
     */
    public double magnitude(){
        return Math.sqrt(Math.pow(this.x, 2)+Math.pow(this.y, 2));
    }
    /**
     * normalizes the point
     * @return the normalized point
     */
    public Point normalize(){
        return this.times(1/this.magnitude());
    }
    /**
     * rotates the point by a given angle
     * @param angle the angle to rotate by
     * @return the rotated point
     */
    public Point rotate(double angle){
        return new Point(this.x*Math.cos(angle)-this.y*Math.sin(angle), this.x*Math.sin(angle)+this.y*Math.cos(angle));
    }
    /**
     * finds the angle of the point
     * @return the angle of the point
     */
    public double angle(){
        return Math.atan2(this.y, this.x);
    }

    /**
     * converts the point to a string
     * @return a point in a string format : {@code "(x,y)"} given x is the points x coordinate and y is the points y coordinate
     */
    @NonNull
    public String toString(){
        return "("+this.x+", "+this.y+")";
    }
}
