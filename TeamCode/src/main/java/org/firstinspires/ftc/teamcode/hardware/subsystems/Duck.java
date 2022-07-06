package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.InterpLUT;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
public class Duck implements Subsystem {

    HardwareMap hwMap;

    DcMotorEx motor;

    public enum MOTOR_STATE {
        RAMP_UP,
        PLATEAU,
        RAMP_OFF,
        STOP,
        IDLE
    }

    MOTOR_STATE state = null;

    public static double rampUpVelo = -500;
    public static double plateauVelo = -600;
    public static double rampOffVelo = -2000;

    public static double rampUpDuration = 0.3;
    public static double plateauDuration = 1.2;
    public static double rampOffDuration = 0.3;

    boolean reset = false;

    Timer fsmTimer = new Timer();

    public Duck(HardwareMap hwMap) {
        this.hwMap = hwMap;
        motor = hwMap.get(DcMotorEx.class, "duck");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startCylce() {
        state = MOTOR_STATE.RAMP_UP;
    }

    public void cancelCycle() {
        state = MOTOR_STATE.STOP;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setActualizedPower(double power) {
        setVelocity(power * 435);
    }

    public void setVelocity(double angularVelo) {
        motor.setVelocity(angularVelo, AngleUnit.DEGREES);
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public void init() throws InterruptedException {
        state = MOTOR_STATE.IDLE;
        resetEncoder();
    }

    @Override
    public void update() throws InterruptedException {
        /*
        switch (state) {
            case RAMP_UP:
                setVelocity(rampUpVelo);

                if(!reset) {
                    fsmTimer.reset();
                    reset = true;
                }

                if(fsmTimer.currentSeconds() > rampUpDuration) {
                    reset = false;
                    state = MOTOR_STATE.PLATEAU;
                }

                break;
            case PLATEAU:
                setVelocity(plateauVelo);

                if(!reset) {
                    fsmTimer.reset();
                    reset = true;
                }

                if(fsmTimer.currentSeconds() > plateauDuration) {
                    reset = false;
                    state = MOTOR_STATE.RAMP_OFF;
                }
                break;
            case RAMP_OFF:
                setVelocity(rampOffVelo);

                if(!reset) {
                    fsmTimer.reset();
                    reset = true;
                }

                if(fsmTimer.currentSeconds() > rampOffDuration) {
                    reset = false;
                    state = MOTOR_STATE.STOP;
                }
                break;
            case STOP:
                setVelocity(0);
                state = MOTOR_STATE.IDLE;
                break;
            case IDLE:
                reset = false;
                fsmTimer.reset();
                break;
            default:
                state = MOTOR_STATE.IDLE;
        }

         */
    }
}
