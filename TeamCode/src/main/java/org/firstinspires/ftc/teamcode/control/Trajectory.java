package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.AngleUtil;

import java.util.ArrayList;

public class Trajectory extends LinearOpMode {
    // TODO: create Holonomic PP point follower w/ heading interpolation and w/o interpolation3
    // TODO: create simple Diffy PP point follower and implent Ramsete if i have time (prolly not)

    ArrayList<Pose2D> path = new ArrayList<>();
    boolean isCompleted = false;
    boolean isStarted = false;

    public Trajectory() {
    }

    public Trajectory(DriveTrain dt, Pose2D end) {
        path.add(dt.localizer.getPose());
        path.add(end);
    }

    public enum controlType {FIELD, ROBOT}

    public Trajectory(ArrayList<Pose2D> path) {
        this.path = path;
    }

    public ArrayList<Pose2D> getPath() {
        return path;
    }

    public void add(Pose2D pose) {
        path.add(pose);
    }

    public void blitz() {
        path.clear();
    }

    public void complete() {
        isCompleted = true;
    }

    public boolean isComplete() {
        return isCompleted;
    }

    public void setStart() {
        isStarted = true;
    }

    public boolean getStart() {
        return isStarted;
    }

    public Pose2D end() {
        return path.get(path.size()-1);
    }
    public static double[] getSegmentLengths(ArrayList<Pose2D> path) {
        double[] lengths = new double[path.size()-1];
        for(int i = 0; i < path.size()-1; i++) {
            lengths[i] = path.get(i).getDistanceFrom(path.get(i+1));
        }

        return lengths;
    }

    public void forward(double distance, controlType style) {
        Pose2D previous = path.get(path.size()-1);
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(path.get(path.size()-1).x, path.get(path.size()-1).y+distance, path.get(path.size()-1).heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(AngleUtil.interpretRadians(previous.heading));
                double y = distance * Math.sin(AngleUtil.interpretRadians(previous.heading));
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    public void backward(double distance, controlType style) {
        Pose2D previous = path.get(path.size()-1);
        distance *= -1;
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(previous.x, previous.y-distance, previous.heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(AngleUtil.interpretRadians(previous.heading));
                double y = distance * Math.sin(AngleUtil.interpretRadians(previous.heading));
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    public void right(double distance, controlType style) {
        Pose2D previous = path.get(path.size()-1);
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(previous.x + distance, previous.y, previous.heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(AngleUtil.interpretRadians(previous.heading + Math.PI/2.0));
                double y = distance * Math.sin(AngleUtil.interpretRadians(previous.heading + Math.PI/2.0));
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    public void left(double distance, controlType style) {
        Pose2D previous = path.get(path.size()-1);
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(previous.x, previous.y+distance, previous.heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(AngleUtil.interpretRadians(previous.heading - Math.PI/2.0));
                double y = distance * Math.sin(AngleUtil.interpretRadians(previous.heading - Math.PI/2.0));
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
