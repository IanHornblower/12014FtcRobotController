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
import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.InterpLUT;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.FreightDetector.*;

// TODO: Add system that sets position with an angle so the Imu can be used to simplify driver control

@Config
public class Arm implements Subsystem {

    HardwareMap hwMap;
    public DcMotorEx turretLift, intake; //, turret;
    public Servo turret, intakeFlap, intakeOrienter;

    public RevColorSensorV3 detector;

    public enum ARM_STATE {
        INTAKING, OUTTAKE, IDLE, PRIMED, DEPOSIT, RETURN, FLOAT
    }

    public ARM_STATE armState = null;

    Timer fsmTimer = new Timer();

    DoubleSupplier manualTurnInput;
    double pointArmPosition;
    public double armPosition;
    double heldPosition;

    public Arm(HardwareMap hwMap) {
        this.hwMap = hwMap;
        turret = hwMap.get(Servo.class, "turret");
        //turret = hwMap.get(DcMotorEx.class, "turret");
        intakeFlap = hwMap.get(Servo.class, "intakeFlap");
        intakeOrienter = hwMap.get(Servo.class, "endTurn");

        intake = hwMap.get(DcMotorEx.class, "intake");
        turretLift = hwMap.get(DcMotorEx.class, "turretLift");

        detector = hwMap.get(RevColorSensorV3.class, "detector");
    }

    public void setState(ARM_STATE state) {
        armState = state;
    }

    public ARM_STATE getState() {
        return armState;
    }

    @Override
    public void init() throws InterruptedException {
        armState = ARM_STATE.IDLE;

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretLift.setDirection(DcMotorSimple.Direction.REVERSE);
        turretLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void update() throws InterruptedException {
        switch (armState) {
            case INTAKING:
                //Orienter Down
                //Intake Spinning
                //Lift pushed all the way down
                break;
            case OUTTAKE:
                // Spit out
                break;

            case IDLE:
                break;
            case PRIMED:
                break;
            case DEPOSIT:
                break;
            case RETURN:
                break;
            case FLOAT:
                break;
        }
    }

    /*
        Lift
     */

    /**
     * Move the arm to the desisted orientation "o"
     * @param orientation
     */
    public void setLiftPosition(double orientation) {
        double currentPositionInDegrees = ArmConstants.Lift.startPosition + convertTurretLiftTicksToRadians(turretLift.getCurrentPosition());

        // Adjust the value of the lift to keep it up when gravity weighs on it the most
        double ff = Math.sin(Math.toRadians(orientation)) * ArmConstants.Lift.ffCoef;

        // Maybe switch orientation & currentPos
        double armPower = ff + ArmConstants.Lift.liftPID.calculate(orientation, currentPositionInDegrees);

        turretLift.setPower(armPower);
    }

    public double convertTurretLiftTicksToRadians(int ticks) {
        return (double) ticks / ArmConstants.Lift.ticksPerDegree;
    }

    public void setIntakePower(double p) {
        if(intake.getPower() != p) {
            intake.setPower(p);
        }
    }

    /*
        Orienter
     */

    public void setIntakeOrientation(double orientation) {
        intakeOrienter.setPosition(orientation);
    }

    /*
        FLAP
     */

    public void setIntakeFlapPosition(double p) {
        intakeFlap.setPosition(p);
    }

    public void openFlap() {
        setIntakeFlapPosition(ArmConstants.Flap.open);
    }

    public void closeFlap() {
        setIntakeFlapPosition(ArmConstants.Flap.closed);
    }

        /*
        Freight Detector
     */

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
}
