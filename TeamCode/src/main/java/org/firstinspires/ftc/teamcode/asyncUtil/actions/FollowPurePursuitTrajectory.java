package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.control.PurePursuit;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.control.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.distanceTolerance;

public class FollowPurePursuitTrajectory extends Action {

    // Action Stuff
    DriveTrain dt;
    Trajectory trajectory;
    double radius;

    // Follower Stuff
    ArrayList<Pose2D> path;
    ArrayList<Pose2D> extendedPath;
    boolean reversed = false;
    int lastIndex;
    Pose2D lastPoint;
    Point pointToFollow;

    public FollowPurePursuitTrajectory(DriveTrain dt, Trajectory trajectory, double radius) {
        this.dt = dt;
        this.trajectory = trajectory;
        this.radius = radius;
    }

    public FollowPurePursuitTrajectory(DriveTrain dt, Trajectory trajectory, double radius, boolean reversed) {
        this.dt = dt;
        this.trajectory = trajectory;
        this.radius = radius;
        this.reversed = reversed;
    }

    @Override
    public void startAction() {
        trajectory.setStart();
        path = trajectory.getPath();
        extendedPath = PurePursuit.extendPath(path, radius);

        lastIndex = path.size()-1;
        lastPoint = path.get(lastIndex);
    }

    @Override
    public void runAction() throws InterruptedException {
        pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, dt, radius);

        double angle = Math.toRadians(270) - Curve.getAngle(pointToFollow, dt.localizer.getPose().toPoint());

        dt.difRunToPosition(pointToFollow.x, pointToFollow.y, angle, reversed);

        error = Math.abs(dt.localizer.getPose().getDistanceFrom(lastPoint)) - radius;

        // Trajectory is Finished if we are within our distance tolerance
        if (error < distanceTolerance) isComplete = true;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
