package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.InterpLUT;

import java.util.function.DoubleSupplier;

// TODO: Add system that sets position with an angle so the Imu can be used to simplify driver control

@Config
public class Arm implements Subsystem {

    HardwareMap hwMap;
    public DcMotorEx turretLift, intake;
    public Servo turret, intakeFlap, intakeOrienter;

    public RevColorSensorV3 detector;

    public enum ARM_STATE {manual, manualPointControl}

    public ARM_STATE armState = null;

    DoubleSupplier manualTurnInput;
    double pointArmPosition;
    public double armPosition;
    double heldPosition;
    public static double leftBound = 0, middle = 0.102, rightBound = 0.212;
    public static double or180 = 0, or90 = 0.49, or0 = 0;
    public static double speed = 0.4;
    public static double distanceTolerance = 50;
    public static double openFlap = 0;
    public static double closedFlap = 0;

    InterpLUT turretLUT = new InterpLUT();
    InterpLUT orienterLUT = new InterpLUT();

    // TODO: When fixing robot -> move to ARM constants file

    // Cube
    public static int lR = 0, lG = 0, lB = 0, lA = 0; // Lower
    public static int uR = 0, uG = 0, uB = 0, uA = 0; // Upper

    double orientation = startPositionInDegrees;

    // Flap
    public static double open = 0, closed = 0.3;

    // Lift Constants
    int ticksPerRev = 3896;
    double ticksPerDegree = ticksPerRev / 360.0;
    public static double startPositionInDegrees = 31;

    public static double kP = 0.015, kI = 0, kD = 0;
    PIDCoefficients liftCoef = new PIDCoefficients(kP, kI, kD);
    BasicPID liftPID = new BasicPID(liftCoef);

    public static double ffCoef = 0.1;

    public Arm(HardwareMap hwMap) {
        this.hwMap = hwMap;
        turret = hwMap.get(Servo.class, "turret");
        intakeFlap = hwMap.get(Servo.class, "intakeFlap");
        intakeOrienter = hwMap.get(Servo.class, "endTurn");

        intake = hwMap.get(DcMotorEx.class, "intake");
        turretLift = hwMap.get(DcMotorEx.class, "turretLift");

        detector = hwMap.get(RevColorSensorV3.class, "detector");
    }

    public void setState(ARM_STATE state) {
        armState = state;
    }

    public void setStartPosition(double turretPos, double armPos) {
        heldPosition = turretPos;
    }

    public void setManualValue(DoubleSupplier manualTurnInput) {
        this.manualTurnInput = manualTurnInput;
    }

    public void setTurretPosition(double p) {
        pointArmPosition = p;
    }

    public double getOrientation() {
        return orientation;
    }

    /**
     * Move the arm to the desisted "p" with a speed
     * @param p
     */
    public void simpleSetLiftPosition(double p, double speed) {
        double error = turretLift.getCurrentPosition() - p;

        double armPower = speed * error/Math.abs(error);

        turretLift.setPower(armPower);
    }

    public boolean hasFreight() {
        return detector.getDistance(DistanceUnit.MM) < distanceTolerance;

    }

    public boolean hasCube() {
        int count = 0;

        float r = detector.getNormalizedColors().alpha;
        float g = detector.getNormalizedColors().green;
        float b = detector.getNormalizedColors().blue;
        float a = detector.getNormalizedColors().alpha;

        if(r > lR && r < uR) count++;
        if(g > lG && g < uG) count++;
        if(b > lB && b < uB) count++;
        if(a > lA && a < uA) count++;

        return count == 4;
    }

    /**
     * Move the arm to the desisted orientation "o"
     * @param orientation
     */
    public void setLiftPosition(double orientation) {
        double currentPositionInDegrees = startPositionInDegrees + convertTurretLiftTicksToRadians(turretLift.getCurrentPosition());

        // Adjust the value of the lift to keep it up when gravity weighs on it the most
        double ff = Math.sin(Math.toRadians(orientation)) * ffCoef;

        // Maybe switch orientation & currentPos
        double armPower = ff + liftPID.calculate(orientation, currentPositionInDegrees);

        turretLift.setPower(armPower);
    }

    public double convertTurretLiftTicksToRadians(int ticks) {
        return (double) ticks / ticksPerDegree;
    }

    public void setIntakePower(double p) {
        if(intake.getPower() != p) {
            intake.setPower(p);
        }
    }

    public void setIntakeOrientation(double orientation) {
        intakeOrienter.setPosition(orientation);
    }

    public void setIntakeFlapPosition(double p) {
        intakeFlap.setPosition(p);
    }

    public void openFlap() {
     setIntakeFlapPosition(openFlap);
    }

    public void closeFlap() {
        setIntakeFlapPosition(closedFlap);
    }

    @Override
    public void init() throws InterruptedException {
        armState = ARM_STATE.manual;

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretLift.setDirection(DcMotorSimple.Direction.REVERSE);
        turretLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretLUT.add(0, leftBound);
        turretLUT.add(180, middle);
        turretLUT.add(360, rightBound);

        orienterLUT.add(0, or0);
        orienterLUT.add(90, or90);
        orienterLUT.add(180, or180);

        turretLUT.createLUT();
        orienterLUT.createLUT();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void update() throws InterruptedException {
        orientation = startPositionInDegrees + convertTurretLiftTicksToRadians(turretLift.getCurrentPosition());


        switch (armState) {
            case manual:
                heldPosition += manualTurnInput.getAsDouble()/(300/speed);
                if(heldPosition > 0.2 || heldPosition < 0) {
                    heldPosition = Range.clip(heldPosition, leftBound, rightBound);
                }
                armPosition = heldPosition;

                if(turret.getPosition() != armPosition) {
                    turret.setPosition(armPosition);
                }
                break;
            case manualPointControl:
                if(turret.getPosition() != pointArmPosition) {
                    turret.setPosition(pointArmPosition);
                }
                break;
            default:
                throw new InterruptedException("State Error");
        }
    }
}
