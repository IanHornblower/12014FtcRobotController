package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.control.EncoderMotionProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
import org.firstinspires.ftc.teamcode.control.OdometricMotionProfile;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.*;

public class DriveTrain implements Subsystem {

    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    double fl, fr, bl, br;

    public DcMotorEx[] motors;

    HardwareMap hwMap;

    public GyroIntegratedThreeWheelOdometry localizer;

    // When odometers are in change to Odometric Profile
    MotionProfile motionProfile = new OdometricMotionProfile(
            xPID, yPID, headingPID, forwardPID, turnPID
    );

    public DriveTrain(HardwareMap hwMap) {
        this.hwMap = hwMap;

        frontLeft = hwMap.get(DcMotorEx.class, "lr"); // should be fl
        frontRight = hwMap.get(DcMotorEx.class, "fr");
        backLeft = hwMap.get(DcMotorEx.class, "bl");
        backRight = hwMap.get(DcMotorEx.class, "br");

        motors = new DcMotorEx[] {frontLeft, frontRight, backLeft, backRight};

        localizer = new GyroIntegratedThreeWheelOdometry(this);
    }

    public void setMotorPowers(double x, double y, double turn) {
        double h = Math.hypot(x, y);
        double theta = Math.atan2(y, x) - Math.toRadians(45);

        double[] motorVector = new double[] {
                (h * Math.cos(theta) + turn),
                (h * Math.sin(theta) - turn),
                (h * Math.sin(theta) + turn),
                (h * Math.cos(theta) - turn)
        };

        setMotorPowers(motorVector[0], motorVector[1],
                       motorVector[2], motorVector[3]);
    }

    public void calculatePosition(double dr, double theta, double turn) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        double mx = cos*dr;
        double my = sin*dr;

        setMotorPowers(mx, my, turn);
    }

    public void driveFieldCentric(double x, double y, double turn) {
        Point vector = new Point(x, y);

        calculatePosition(
                vector.hypot(),
                vector.atan2() + localizer.imu.getHeadingInRadians() - Math.toRadians(90),
                turn);
    }

    public void driveFieldCentric(double x, double y, double turn, double modHeading) {
        Point vector = new Point(x, y);

        calculatePosition(
                vector.hypot(),
                vector.atan2() - localizer.imu.getHeadingInRadians() + modHeading,
                turn);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    public void stopDriveTrain() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void emergencyStop() {
        for(DcMotorEx m : motors) {
            m.setPower(0.0);
            m.setMotorDisable();
        }
    }

    public HardwareMap getHardwareMap() {
        return hwMap;
    }

    public void setStartPosition(Pose2D pose) {
        localizer.setStartPosition(pose);
    }

    public void resetEncoders() {
        for(DcMotorEx motors:motors) {
            motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setWeightedDrivePower(double x, double y, double heading) {
        Pose2D vel = new Pose2D(x, y, heading);

        if (Math.abs(x) + Math.abs(y)
                + Math.abs(heading) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(x)
                    + VY_WEIGHT * Math.abs(y)
                    + TURN_WEIGHT * Math.abs(heading);

            vel = new Pose2D(
                    VX_WEIGHT * x,
                    VY_WEIGHT * y,
                    TURN_WEIGHT * heading
            ).div(denom);
        }

        setMotorPowers(vel.x, vel.y, vel.heading);
    }

    public void setFieldCentricDrivePower(double x, double y, double heading) {
        Pose2D vel = new Pose2D(x, y, heading);

        if (Math.abs(x) + Math.abs(y)
                + Math.abs(heading) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(x)
                    + VY_WEIGHT * Math.abs(y)
                    + TURN_WEIGHT * Math.abs(heading);

            vel = new Pose2D(
                    VX_WEIGHT * x,
                    VY_WEIGHT * y,
                    TURN_WEIGHT * heading
            ).div(denom);
        }

        driveFieldCentric(vel.x, vel.y, vel.heading);
    }

    public void runToPosition(double x, double y, double heading) throws InterruptedException {
        motionProfile.runToPosition(this, x, y, heading);
    }

    public void difRunToPosition(double x, double y, boolean reversed) throws InterruptedException {
        motionProfile.difRunToPosition(this, x, y, reversed);
    }

    public void difRunToPosition(double x, double y, double heading, boolean reversed) throws InterruptedException {
        motionProfile.difRunToPosition(this, x, y, heading, reversed);
    }

    public void rotate(double heading) throws InterruptedException {
        motionProfile.rotate(this, heading);
    }

    @Override
    public void init() throws InterruptedException {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for(DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        resetEncoders();

        // Will be 3 diffrent motors
        localizer.initDoubleSuppliers(
                ()-> motors[0].getCurrentPosition(),  // LEFT
                ()-> motors[1].getCurrentPosition(),  // RIGHT
                ()-> motors[2].getCurrentPosition() // LATERAL
        );

        localizer.setConstants(12, 1120, 1, 0.0);
        localizer.setMeasurement(GyroIntegratedThreeWheelOdometry.inputMeasurement.INCH);

        localizer.setKalmanConstants(0.0, 1, 3, 1, 0.0);

        localizer.init();
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void update() throws InterruptedException {
        localizer.update();

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
