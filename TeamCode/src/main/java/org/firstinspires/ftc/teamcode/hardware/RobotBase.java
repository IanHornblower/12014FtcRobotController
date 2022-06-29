package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.gamepad.Joystick;
import org.firstinspires.ftc.teamcode.hardware.subsystems.*;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Subsystem;

public class RobotBase extends OpMode implements Robot {

    public SampleMecanumDrive driveTrain;
    public Duck duck;
    public Arm arm;

    Subsystem[] subsystems = {};

    public GamepadEx driverGamepad;
    public GamepadEx operatorGamepad;

    public RobotBase (HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2) {
        driveTrain = new SampleMecanumDrive(hwMap);
        duck = new Duck(hwMap);
        arm = new Arm(hwMap);

        subsystems = new Subsystem[] {driveTrain, duck, arm};

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
    }

    @Override
    public void initHardwareMap() throws InterruptedException {
        for (Subsystem subsystem : subsystems){
            subsystem.init();
        }
    }

    @Override
    public void update() throws InterruptedException {
        for (Subsystem subsystem : subsystems){
            subsystem.update();
        }
    }

    @Override
    public void resetEncoders() {
        duck.resetEncoder();
    }

    @Override
    public void emergencyStop() {
        driveTrain.setMotorPowers(0, 0, 0, 0);
    }

    @Override
    public void init() {
    }

    @Override
    public void loop() {

    }

}
