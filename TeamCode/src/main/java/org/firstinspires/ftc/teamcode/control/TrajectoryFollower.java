package org.firstinspires.ftc.teamcode.control;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;

// TODO: new system in place for ASYNC travel this will be removed and replaced with individual actions

public class TrajectoryFollower extends LinearOpMode {

    DriveTrain dt;
    double timeout;
    double distanceTolerance;

    ArrayList<Pose2D> path;
    ArrayList<Pose2D> extendedPath;
    int lastIndex;
    Pose2D lastPoint;
    Point pointToFollow;

    Trajectory currentTrajectory;

    Timer timer = new Timer();

    boolean init = false;
    boolean isCompleted = false;
    boolean reversed = false;
    int closestPointIndex = 0;
    double runningCount = 1e+6;

    double[] segmentLength;
    int index = 0;

    public enum STATE {START, RUNNING, COMPLETED, STOPPED}
    STATE state;

    public TrajectoryFollower(DriveTrain dt, double timeout, double distanceTolerance) {
        this.dt = dt;
        this.timeout = timeout;
        this.distanceTolerance = distanceTolerance;

        state = STATE.START;
    }

    public boolean isCompleted() {
        return isCompleted;
    }

    public Trajectory currentTrajectory() {
        return currentTrajectory;
    }

    public void setTrajectory(Trajectory trajectory) {
        currentTrajectory = trajectory;
        state = STATE.START;
        isCompleted = false;
        timer.start();
    }

    public void setState(STATE s) {
        state = s;
    }

    public STATE state() {
        return state;
    }

    public void reverse() {
        reversed = true;
    }

    public void forward() {
        reversed = true;
    }

    public void HolonomicFollower(double radius) throws InterruptedException {
        switch (state) {
            case START:
                currentTrajectory.setStart();
                path = currentTrajectory.getPath();
                extendedPath = PurePursuit.extendPath(path, radius);

                lastIndex = path.size()-1;
                lastPoint = path.get(lastIndex);
                state = STATE.RUNNING;
                timer.start();
                break;
            case RUNNING:
                pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, dt, radius);

                dt.runToPosition(pointToFollow.x, pointToFollow.y, path.get(closestPointIndex).heading);

                double error = Math.abs(dt.localizer.getPose().getDistanceFrom(lastPoint)) - radius;

                // TODO: Change to after point change, current system does it half way (closest System)
                timer.reset();
                for(int i = 0; i < path.size(); i++) {
                    double distanceTo = dt.localizer.getPose().getDistanceFrom(path.get(i));

                    if(distanceTo < runningCount) {
                        closestPointIndex = i;
                        runningCount = distanceTo;
                    }
                }
                //System.out.println(timer.currentSeconds());

                runningCount = 1e+6;

                // Trajectory is Finished if the timeout has been reached, or we are within our distance tolerance
                if((timer.currentSeconds() > timeout) || (error < distanceTolerance)) state = STATE.COMPLETED;
                break;
            case COMPLETED:
                currentTrajectory.complete();
                isCompleted = true;
                state = STATE.STOPPED;
                break;
            case STOPPED:
                dt.stopDriveTrain();
                break;
        }

    }

    public void AnglePursuitFollower(double radius) throws InterruptedException {
        switch (state) {
            case START:
                currentTrajectory.setStart();
                path = currentTrajectory.getPath();
                extendedPath = PurePursuit.extendPath(path, radius);

                lastIndex = path.size()-1;
                lastPoint = path.get(lastIndex);
                state = STATE.RUNNING;
                break;
            case RUNNING:
                pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, dt, radius);

                double angle = Math.toRadians(270)-Curve.getAngle(pointToFollow, dt.localizer.getPose().toPoint());

                dt.difRunToPosition(pointToFollow.x, pointToFollow.y, angle, reversed);

                double error = Math.abs(dt.localizer.getPose().getDistanceFrom(lastPoint)) - radius;

                // Trajectory is Finished if the timeout has been reached, or we are within our distance tolerance
                if((timer.currentSeconds() > timeout) || (error < distanceTolerance)) state = STATE.COMPLETED;
                break;
            case COMPLETED:
                currentTrajectory.complete();
                isCompleted = true;
                state = STATE.STOPPED;
                break;
            case STOPPED:
                dt.stopDriveTrain();
                break;
        }

    }

    public void SetHeadingFollower(double radius, double heading) throws InterruptedException {
        switch (state) {
            case START:
                currentTrajectory.setStart();
                path = currentTrajectory.getPath();
                extendedPath = PurePursuit.extendPath(path, radius);

                lastIndex = path.size()-1;
                lastPoint = path.get(lastIndex);
                state = STATE.RUNNING;
                break;
            case RUNNING:
                pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, dt, radius);

                dt.runToPosition(pointToFollow.x, pointToFollow.y, heading);

                double error = Math.abs(dt.localizer.getPose().getDistanceFrom(lastPoint)) - radius;

                // Trajectory is Finished if the timeout has been reached, or we are within our distance tolerance
                if((timer.currentSeconds() > timeout) || (error < distanceTolerance)) state = STATE.COMPLETED;
                break;
            case COMPLETED:
                currentTrajectory.complete();
                isCompleted = true;
                state = STATE.STOPPED;
                break;
            case STOPPED:
                dt.stopDriveTrain();
                break;
        }

    }

    public void actualHolonomicFollower(double radius) throws InterruptedException {
        switch (state) {
            case START:
                currentTrajectory.setStart();
                path = currentTrajectory.getPath();
                extendedPath = PurePursuit.extendPath(path, radius);

                segmentLength = new double[path.size()-1];

                for(int i = 0; i < path.size()-1; i++) {
                    segmentLength[i] = path.get(i).getDistanceFrom(path.get(i+1));
                }

                lastIndex = path.size()-1;
                lastPoint = path.get(lastIndex);
                state = STATE.RUNNING;
                timer.start();
                break;
            case RUNNING:
                pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, dt, radius);

                dt.runToPosition(pointToFollow.x, pointToFollow.y, path.get(index).heading);

                double error = Math.abs(dt.localizer.getPose().getDistanceFrom(lastPoint)) - radius;

                if(dt.localizer.accumulatedDistance > segmentLength[index]) {
                    dt.localizer.resetAccumulatedDistance();
                    index++;
                }

                // Trajectory is Finished if the timeout has been reached, or we are within our distance tolerance
                if((timer.currentSeconds() > timeout) || (error < distanceTolerance)) state = STATE.COMPLETED;
                break;
            case COMPLETED:
                currentTrajectory.complete();
                isCompleted = true;
                state = STATE.STOPPED;
                break;
            case STOPPED:
                dt.stopDriveTrain();
                break;
        }

    }

    public void tightHolonomicFollower(double radius) throws InterruptedException {
        switch (state) {
            case START:
                currentTrajectory.setStart();
                path = currentTrajectory.getPath();
                extendedPath = PurePursuit.extendPath(path, radius);

                segmentLength = new double[path.size()-1];

                for(int i = 0; i < path.size()-1; i++) {
                    segmentLength[i] = path.get(i).getDistanceFrom(path.get(i+1));
                }

                lastIndex = path.size()-1;
                lastPoint = path.get(lastIndex);
                state = STATE.RUNNING;
                timer.start();
                break;
            case RUNNING:
                if(dt.localizer.accumulatedDistance > segmentLength[index]) {
                    dt.localizer.resetAccumulatedDistance();
                    index++;
                }

                dt.runToPosition(path.get(index+1).x, path.get(index+1).y, path.get(index+1).heading);

                double error = Math.abs(dt.localizer.getPose().getDistanceFrom(lastPoint)) - radius;

                // Trajectory is Finished if the timeout has been reached, or we are within our distance tolerance
                if((timer.currentSeconds() > timeout) || (error < distanceTolerance)) state = STATE.COMPLETED;
                break;
            case COMPLETED:
                currentTrajectory.complete();
                isCompleted = true;
                state = STATE.STOPPED;
                break;
            case STOPPED:
                dt.stopDriveTrain();
                break;
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
