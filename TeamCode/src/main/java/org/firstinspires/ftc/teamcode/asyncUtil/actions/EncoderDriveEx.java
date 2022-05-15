package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.hardware.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;

public class EncoderDriveEx extends Action {

    DriveTrain dt;
    double x, y, heading;
    int ticks;

    double[] totalTicks = new double[4];

    boolean fieldCentric;

    BasicPID headingPID = new BasicPID(DriveConstants.encoderHeadingPID);
    AngleController headingController = new AngleController(headingPID);

    public EncoderDriveEx(DriveTrain dt, double x, double y, double heading, int ticks) {
        this.dt = dt;
        this.ticks = ticks;
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public EncoderDriveEx(DriveTrain dt, double x, double y, double heading, int ticks, boolean fieldCentric) {
        this.dt = dt;
        this.ticks = ticks;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void startAction() {
    }

    @Override
    public void runAction() throws InterruptedException {
        if(fieldCentric) {
            dt.driveFieldCentric(x, y, headingController.calculate(heading, dt.localizer.imu.getHeadingInRadians()));
        }
        else {
            dt.setMotorPowers(x, y, headingController.calculate(heading, dt.localizer.imu.getHeadingInRadians()));
        }

        for(int i = 0; i < 4; i++) {
            totalTicks[i] += Math.abs(dt.motors[i].getCurrentPosition())/(double)DriveConstants.EncoderTicks;
            isComplete =  totalTicks[i] > ticks;
        }
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
