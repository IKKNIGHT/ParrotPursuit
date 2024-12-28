package org.firstinspires.ftc.teamcode.paths;

import androidx.annotation.NonNull;

public class Point {
    private double x, y;
    public Point(double x, double y){
        this.x=x;
        this.y=y;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public Point times(double amount){
        return new Point(x*amount, y*amount);
    }
    public Point add(Point other){
        return new Point(this.x+other.getX(), this.y+other.getY());
    }
    public Point subtract(Point other){
        return new Point(this.x-other.getX(), this.y-other.getY());
    }
    public double distance(Point other){
        return Math.sqrt(Math.pow(this.x-other.getX(), 2)+Math.pow(this.y-other.getY(), 2));
    }
    public double dot(Point other){
        return this.x*other.getX()+this.y*other.getY();
    }
    public double cross(Point other){
        return this.x*other.getY()-this.y*other.getX();
    }
    public double magnitude(){
        return Math.sqrt(Math.pow(this.x, 2)+Math.pow(this.y, 2));
    }
    public Point normalize(){
        return this.times(1/this.magnitude());
    }
    public Point rotate(double angle){
        return new Point(this.x*Math.cos(angle)-this.y*Math.sin(angle), this.x*Math.sin(angle)+this.y*Math.cos(angle));
    }
    public double angle(){
        return Math.atan2(this.y, this.x);
    }
    @NonNull
    public String toString(){
        return "("+this.x+", "+this.y+")";
    }
}
