package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;

public class EncoderMotionProfile implements MotionProfile {

    @Override
    public void rotate(DriveTrain dt, double heading) throws InterruptedException {

    }

    @Override
    public void encoderDrive(DriveTrain dt, double x, double y, double heading, boolean fieldCentric) throws InterruptedException {

    }

    @Override
    public void difRunToPosition(DriveTrain dt, double x, double y, double heading, boolean reversed) throws InterruptedException {
        throw new InterruptedException("Wrong Motion Profile Try Using Odometers, Currently: ENCODER MOTION PROFILE");
    }

    @Override
    public void difRunToPosition(DriveTrain dt, double x, double y, boolean reversed) throws InterruptedException {
        throw new InterruptedException("Wrong Motion Profile Try Using Odometers, Currently: ENCODER MOTION PROFILE");
    }

    @Override
    public void runToPosition(DriveTrain dt, double x, double y, double heading) throws InterruptedException {
        throw new InterruptedException("Wrong Motion Profile Try Using Odometers, Currently: ENCODER MOTION PROFILE");
    }
}
