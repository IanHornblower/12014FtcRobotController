package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.hardware.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.*;

public class RunToPosition extends Action {

    DriveTrain dt;
    Pose2D endPos;

    BasicPID xPID, yPID, headingPID;
    AngleController headingController;

    public RunToPosition(DriveTrain dt, Pose2D endPos) {
        this.dt = dt;
        this.endPos = endPos;

        xPID = new BasicPID(DriveConstants.xPID);
        yPID = new BasicPID(DriveConstants.yPID);
        headingPID = new BasicPID(DriveConstants.headingPID);

        headingController = new AngleController(headingPID);
    }

    @Override
    public void startAction() {
    }

    @Override
    public void runAction() throws InterruptedException {
        Pose2D pos = dt.localizer.getPose();

        double xP = xPID.calculate(endPos.x, pos.x);
        double yP = yPID.calculate(endPos.y, pos.y);
        double headingP = headingController.calculate(endPos.heading, pos.heading);

        dt.driveFieldCentric(xP, yP, headingP);

        error = dt.localizer.getPose().getDistanceFrom(pos.toPoint());
        isComplete = error < distanceTolerance && dt.getCombinedVelocity() < 0.5;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
