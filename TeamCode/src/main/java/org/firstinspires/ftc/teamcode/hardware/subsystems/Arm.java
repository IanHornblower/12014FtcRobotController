package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Subsystem;

import java.util.function.DoubleSupplier;

// TODO: Add system that sets position with an angle so the Imu can be used to simplify driver control

@Config
public class Arm implements Subsystem {

    HardwareMap hwMap;
    public DcMotorEx turretLift, intake;
    public Servo turret, intakeFlap;

    public enum ARM_STATE {manual, manualPointControl}

    public ARM_STATE armState = null;

    DoubleSupplier manualTurnInput;
    double pointArmPosition;
    public double armPosition;
    double heldPosition;
    public static double leftBound = 0, middle = 0.102, rightBound = 0.212;
    public static double speed = 0.4;

    public Arm(HardwareMap hwMap) {
        this.hwMap = hwMap;
        turret = hwMap.get(Servo.class, "turret");
        intakeFlap = hwMap.get(Servo.class, "intakeFlap");

        intake = hwMap.get(DcMotorEx.class, "intake");
        turretLift = hwMap.get(DcMotorEx.class, "turretLift");
    }

    public void setState(ARM_STATE state) {
        armState = state;
    }

    public void setManualValue(DoubleSupplier manualTurnInput) {
        this.manualTurnInput = manualTurnInput;
    }

    public void setPointArmPosition(double p) {
        pointArmPosition = p;
    }

    @Override
    public void init() throws InterruptedException {
        armState = ARM_STATE.manualPointControl;
        heldPosition = middle;

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretLift.setDirection(DcMotorSimple.Direction.FORWARD);
        turretLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void update() throws InterruptedException {
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
