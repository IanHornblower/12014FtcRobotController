package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.control.PurePursuit;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.control.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.distanceTolerance;

public class FollowSetHeadingHolonomicTrajectory extends Action {

    // Action Stuff
    DriveTrain dt;
    Trajectory trajectory;
    double heading;
    double radius;

    // Follower Stuff
    ArrayList<Pose2D> path;
    ArrayList<Pose2D> extendedPath;
    boolean reversed = false;
    int lastIndex;
    Pose2D lastPoint;
    Point pointToFollow;

    public FollowSetHeadingHolonomicTrajectory(DriveTrain dt, Trajectory trajectory, double radius, double heading) {
        this.dt = dt;
        this.trajectory = trajectory;
        this.radius = radius;
        this.heading = heading;
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

        dt.runToPosition(pointToFollow.x, pointToFollow.y, heading);

        double error = Math.abs(dt.localizer.getPose().getDistanceFrom(lastPoint)) - radius;

        if(error < distanceTolerance) isComplete = true;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
