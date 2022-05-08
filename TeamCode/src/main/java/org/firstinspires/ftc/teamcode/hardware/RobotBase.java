package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.gamepad.Joystick;
import org.firstinspires.ftc.teamcode.hardware.subsystems.*;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Subsystem;

public class RobotBase extends OpMode implements Robot {

    public DriveTrain driveTrain;
    public Duck duck;

    Subsystem[] subsystems = {};

    public GamepadEx driverGamepad;
    public GamepadEx operatorGamepad;

    public RobotBase (HardwareMap hwMap) {
        driveTrain = new DriveTrain(hwMap);
        duck = new Duck(hwMap);

        subsystems = new Subsystem[] {driveTrain, duck};

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        driverGamepad.leftJoystick.invert(Joystick.AXIS.y);
        driverGamepad.rightJoystick.invert(Joystick.AXIS.y);
        operatorGamepad.leftJoystick.invert(Joystick.AXIS.y);
        operatorGamepad.rightJoystick.invert(Joystick.AXIS.y);
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
        driveTrain.resetEncoders();
        duck.resetEncoder();
    }

    @Override
    public void emergencyStop() {
        driveTrain.emergencyStop();
    }

    @Override
    public void init() {
    }

    @Override
    public void loop() {

    }

}
