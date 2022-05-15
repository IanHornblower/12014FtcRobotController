package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Curve;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.rotationTolerance;

public class IMURotate extends Action {

    DriveTrain dt;
    double heading;

    public IMURotate(DriveTrain dt, double heading) {
        this.dt = dt;
        this.heading = heading;
    }

    @Override
    public void startAction() {

    }

    @Override
    public void runAction() throws InterruptedException {
        dt.rotate(heading);
        error = Curve.getShortestDistance(heading, dt.localizer.imu.getHeadingInRadians());
        isComplete = error < rotationTolerance;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }



}
