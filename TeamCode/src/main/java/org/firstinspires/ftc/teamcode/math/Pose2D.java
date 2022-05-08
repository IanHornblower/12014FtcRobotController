package org.firstinspires.ftc.teamcode.math;

import org.firstinspires.ftc.teamcode.util.AngleUtil;

import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

public class Pose2D {

    public double x, y, heading;
    public double xVelocity, yVelocity, headingVelocity;

    public Pose2D (double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2D (Point point, double heading) {
        x = point.x;
        y = point.y;
        this.heading = heading;
    }

    public void invertPose() {
        double tempX = x, tempY = y;
        x = tempY;
        y = tempX;
    }

    public void scalePose(double scaleFactor) {
        x *= scaleFactor;
        y *= scaleFactor;
    }

    public double getHeading() {
        return heading;
    }

    public Pose2D div(double n) {
        return new Pose2D(x/n, y/n, heading/n);
    }

    public double getHeadingInDegrees() {
        return Math.toDegrees(getHeading());
    }

    public void addPoint(Point p) {
        x += p.x;
        y += p.y;
    }

    public void addPose2D(Pose2D p) {
        x += p.x;
        y += p.y;
        heading += p.heading;
    }

    public double getY() {
        return y;
    }

    public double getX() {
        return x;
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    public String toString() {
        return String.format(
                roundPlaces(x, 1) +
                        " " + roundPlaces(y, 1) +
                        " " + roundPlaces(Math.toDegrees(heading), 1));
    }

    public double getDistanceFrom(Point point) {
        return getDistance(this, point);
    }

    public double getDistanceFrom(Pose2D pose) {
        return getDistance(this, pose.toPoint());
    }

    public static double getDistance(Pose2D start, Point end) {
        return Math.sqrt(Math.pow((end.getX()-start.getX()),2)+Math.pow((end.getY()-start.getY()),2));
    }
}