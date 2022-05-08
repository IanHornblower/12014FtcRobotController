package org.firstinspires.ftc.teamcode.control;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Point;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import org.firstinspires.ftc.teamcode.math.Pose2D;

public class OdometricMotionProfile implements MotionProfile {

    PIDCoefficients xPIDCoef, yPIDCoef, headingPIDCoef, forwardPIDCoef, turnPIDCoef;
    BasicPID xPID, yPID, headingPID, forwardPID, turnPID;
    AngleController headingController;
    AngleController turnController;

    public OdometricMotionProfile(PIDCoefficients xPIDCoef, PIDCoefficients yPIDCoef, PIDCoefficients headingPIDCoef, PIDCoefficients forwardPIDCoef, PIDCoefficients turnPIDCoef) {
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

    public void runToPosition(DriveTrain dt, double x, double y, double heading) {
        Pose2D pos = dt.localizer.getPose();

        double xP = xPID.calculate(x, pos.x);
        double yP = yPID.calculate(y, pos.y);
        double headingP = headingController.calculate(heading, pos.heading);

        dt.driveFieldCentric(xP, yP, headingP);
    }

    public void difRunToPosition(DriveTrain dt, double x, double y, boolean reversed) {
        Pose2D pos = dt.localizer.getPose();

        double xError = x - pos.x;
        double yError = y - pos.y;
        double theta = Math.atan2(yError, xError);
        if(reversed) theta = Math.atan2(-yError, -xError);

        double hypot = pos.getDistanceFrom(new Point(x, y));

        double f = -forwardPID.calculate(0, hypot);
        if(reversed) f *= -1;
        double t = turnController.calculate(theta, pos.heading);

        dt.setMotorPowers(
                f + t,
                f - t,
                f + t,
                f - t
        );
    }

    public void difRunToPosition(DriveTrain dt, double x, double y, double heading, boolean reversed) {
        Pose2D pos = dt.localizer.getPose();

        double hypot = pos.getDistanceFrom(new Point(x, y));

        double f = -forwardPID.calculate(0, hypot);
        if(reversed) f *= -1;
        double angle = pos.heading;
        if(reversed) angle += Math.toRadians(180);

        double t = turnController.calculate(heading, angle);

        dt.setMotorPowers(
                f + t,
                f - t,
                f + t,
                f - t
        );
    }

    public void rotate(DriveTrain dt, double heading) {
        Pose2D pos = dt.localizer.getPose();

        double headingP = headingController.calculate(heading, pos.heading);
        dt.driveFieldCentric(0, 0, headingP);
    }

    @Override
    public void encoderDrive(DriveTrain dt, double x, double y, double heading, boolean fieldCentric) throws InterruptedException {
    }


}
