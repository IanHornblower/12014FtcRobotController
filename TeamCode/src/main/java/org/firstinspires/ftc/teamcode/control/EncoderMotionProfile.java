package org.firstinspires.ftc.teamcode.control;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;

public class EncoderMotionProfile implements MotionProfile {

    PIDCoefficients xPIDCoef, yPIDCoef, headingPIDCoef, forwardPIDCoef, turnPIDCoef;
    BasicPID xPID, yPID, headingPID, forwardPID, turnPID;
    AngleController headingController;
    AngleController turnController;

    public EncoderMotionProfile(PIDCoefficients xPIDCoef, PIDCoefficients yPIDCoef, PIDCoefficients headingPIDCoef, PIDCoefficients forwardPIDCoef, PIDCoefficients turnPIDCoef) {
        this.xPIDCoef = xPIDCoef;
        this.yPIDCoef = yPIDCoef;
        this.headingPIDCoef = headingPIDCoef;
        this.forwardPIDCoef = forwardPIDCoef;
        this.turnPIDCoef = turnPIDCoef;

        xPID = new BasicPID(xPIDCoef);
        yPID = new BasicPID(yPIDCoef);
        headingPID = new BasicPID(headingPIDCoef);
        forwardPID = new BasicPID(forwardPIDCoef);
        turnPID = new BasicPID(turnPIDCoef);
        headingController = new AngleController(headingPID);
        turnController = new AngleController(turnPID);
    }

    @Override
    public void rotate(DriveTrain dt, double heading) throws InterruptedException {
        double robotHeading = dt.localizer.imu.getHeadingInRadians();
        double headingP = headingController.calculate(heading, robotHeading);

        dt.driveFieldCentric(0, 0, headingP);
    }

    @Override
    public void encoderDrive(DriveTrain dt, double x, double y, double heading, boolean fieldCentric) throws InterruptedException {
        // TODO: Do later
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
