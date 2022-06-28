package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.control.PurePursuit;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.distanceTolerance;

public class FollowActualTrajectory extends Action {

    // Action Stuff
    DriveTrain dt;
    Trajectory trajectory;

    // Follower Stuff
    ArrayList<Pose2D> path;
    double[] lengths;
    int lastIndex;
    Pose2D lastPoint;

    BasicPID xPID = new BasicPID(DriveConstants.xPID);
    BasicPID yPID = new BasicPID(DriveConstants.yPID);
    BasicPID headingPID = new BasicPID(DriveConstants.headingPID);
    AngleController headingController = new AngleController(headingPID);
    BasicPID distanceController = new BasicPID(DriveConstants.trajectoryPID);

    public FollowActualTrajectory(DriveTrain dt, Trajectory trajectory) {
        this.dt = dt;
        this.trajectory = trajectory;
    }

    @Override
    public void startAction() {
        dt.localizer.resetAccumulatedDistance();

        trajectory.setStart();
        path = trajectory.getPath();

        lengths = Trajectory.getSegmentLengths(path);

        lastIndex = path.size()-1;
        lastPoint = path.get(lastIndex);
    }

    // Start at first point
    int index = 1;

    double xP, yP, distanceP, angle, headingP;

    @Override
    public void runAction() throws InterruptedException {
        Pose2D pos = dt.localizer.getPose();

        double error = Math.abs(pos.getDistanceFrom(lastPoint));

        if(dt.localizer.accumulatedDistance > lengths[index-1]) {
            dt.localizer.resetAccumulatedDistance();
            index++;
        }

        if(index == path.size()-2) {
            headingP = headingController.calculate(path.get(index).heading, pos.heading);

            xP = xPID.calculate(path.get(index).x, pos.x);
            yP = yPID.calculate(path.get(index).y, pos.y);
        }
        else {
            headingP = headingController.calculate(path.get(index).heading, pos.heading);

            distanceP = distanceController.calculate(error, 0);
            angle = Math.atan2(yPID.calculate(path.get(index).y, pos.y), xPID.calculate(path.get(index).x, pos.x));
            xP = Math.cos(angle) * distanceP;
            yP = Math.sin(angle) * distanceP;
        }

        dt.driveFieldCentric(xP, yP, headingP);

        if(error < distanceTolerance && dt.getCombinedVelocity() < 0.5) isComplete = true;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
