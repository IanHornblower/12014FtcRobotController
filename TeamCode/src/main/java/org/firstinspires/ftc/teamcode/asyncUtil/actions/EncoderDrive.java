package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.hardware.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;

public class EncoderDrive extends Action {

    DriveTrain dt;
    double x, y, h;
    int ticks;

    boolean fieldCentric;

    double[] totalTicks = new double[4];


    public EncoderDrive(DriveTrain dt, double x, double y, double h, int ticks) {
        this.dt = dt;
        this.ticks = ticks;
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public EncoderDrive(DriveTrain dt, double x, double y, double h, int ticks, boolean fieldCentric) {
        this.dt = dt;
        this.ticks = ticks;
        this.x = x;
        this.y = y;
        this.h = h;
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void startAction() {
    }

    @Override
    public void runAction() throws InterruptedException {
        if(fieldCentric) {
            dt.driveFieldCentric(x, y, h);
        }
        else {
            dt.setMotorPowers(x, y, h);
        }

        for(int i = 0; i < 4; i++) {
            totalTicks[i] += Math.abs(dt.motors[i].getCurrentPosition())/(double) DriveConstants.EncoderTicks;
            isComplete =  totalTicks[i] > ticks;
        }
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
