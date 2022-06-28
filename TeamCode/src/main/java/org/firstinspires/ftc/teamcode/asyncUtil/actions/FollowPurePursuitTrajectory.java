package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.control.PurePursuit;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.control.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.hardware.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.*;

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

    BasicPID forwardPID, turnPID;
    AngleController turnController;

    public FollowPurePursuitTrajectory(DriveTrain dt, Trajectory trajectory, double radius) {
        this.dt = dt;
        this.trajectory = trajectory;
        this.radius = radius;

        forwardPID = new BasicPID(DriveConstants.forwardPID);
        turnPID = new BasicPID(DriveConstants.turnPID);

        turnController = new AngleController(turnPID);
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

    double f = 0;
    double t = 0;

    @Override
    public void runAction() throws InterruptedException {
        pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, dt, radius);
        Pose2D pos = dt.localizer.getPose();

        double angle = Math.toRadians(270) - Curve.getAngle(pointToFollow, pos.toPoint());

        double hypot = pos.getDistanceFrom(new Point(pointToFollow.x, pointToFollow.y));

        error = Math.abs(pos.getDistanceFrom(lastPoint)) - radius;

        f = forwardPID.calculate(hypot, 0);
        if(reversed) f *= -1;
        if(reversed) angle += Math.toRadians(180);

        t = turnController.calculate(angle, pos.heading);

        // Cuz code in inverted use sine instead of cosine
        f *= Math.sin(Range.clip(error, -Math.PI/2, Math.PI/2));

        dt.setMotorPowers(
            f + t,
            f - t,
            f + t,
            f - t
        );

        // Trajectory is Finished if we are within our distance tolerance
        if (error < distanceTolerance && dt.getCombinedVelocity() < 0.5) isComplete = true;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
