package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;

public interface MotionProfile {

    void rotate(DriveTrain dt, double heading) throws InterruptedException;

    void encoderDrive(DriveTrain dt, double x, double y, double heading, boolean fieldCentric) throws InterruptedException;

    void runToPosition(DriveTrain dt, double x, double y, double heading) throws InterruptedException;

    void difRunToPosition(DriveTrain dt, double x, double y, double heading, boolean reversed) throws InterruptedException;

    void difRunToPosition(DriveTrain dt, double x, double y, boolean reversed) throws InterruptedException;
}
