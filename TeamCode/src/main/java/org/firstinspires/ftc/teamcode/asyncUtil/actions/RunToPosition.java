package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.*;

public class RunToPosition extends Action {

    DriveTrain dt;
    Pose2D pos;

    public RunToPosition(DriveTrain dt, Pose2D pos) {
        this.dt = dt;
        this.pos = pos;
    }

    @Override
    public void startAction() {
    }

    @Override
    public void runAction() throws InterruptedException {
        dt.runToPosition(pos.x, pos.y, pos.heading);
        error = dt.localizer.getPose().getDistanceFrom(pos.toPoint());
        isComplete = error < distanceTolerance;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
