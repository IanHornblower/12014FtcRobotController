package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.middle;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.orUP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.control.*;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;

@TeleOp(name = "Actual TeleOP", group = "yes")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotBase actualRobot = new RobotBase(hardwareMap, gamepad1, gamepad2);

        actualRobot.initHardwareMap();
        actualRobot.driveTrain.setStartPosition(new Pose2D(0, 0, Math.toRadians(0)));

        actualRobot.arm.setStartPosition(middle, 0);
        actualRobot.arm.setManualValue(()-> actualRobot.operatorGamepad.leftJoystick.x());

        Timer armTime = new Timer();

        double armPos = Arm.startPositionInDegrees;
        double orienterPos = Arm.orRegular;

        waitForStart();

        while (opModeIsActive()) {
            /*
                DRIVE TRAIN
             */

            double x = actualRobot.driverGamepad.leftJoystick.x();
            double y = actualRobot.driverGamepad.leftJoystick.y();
            double turn = actualRobot.driverGamepad.rightJoystick.x();

            actualRobot.driveTrain.setWeightedDrivePower(x, y, turn);

            /*
                ARM
             */

            actualRobot.arm.setIntakePower(-actualRobot.operatorGamepad.rightTrigger.value()+actualRobot.operatorGamepad.leftTrigger.value());

            //if(actualRobot.operatorGamepad.leftTrigger.isPressed() || actualRobot.operatorGamepad.rightTrigger.isPressed()) {
            //    actualRobot.arm.setIntakeOrientation(0.53);
            //}

            switch (actualRobot.operatorGamepad.dpad()) {
                case up:
                    armPos = 190;
                    orienterPos = 0.2;
                    break;
                case down:
                    armPos = 35;
                    orienterPos = 0.4;
                    armTime.reset();
                    if(armTime.currentSeconds() > 0.5) {
                        armPos = 35;
                    }
                    break;
                case left:
                    orienterPos = 0.49;
                default:

                    break;
            }

            actualRobot.arm.setLiftPosition(armPos);
            actualRobot.arm.setIntakeOrientation(orienterPos);

            if(actualRobot.operatorGamepad.leftJoystick.isPressed()) {
                actualRobot.arm.armPosition = middle;
            }


            actualRobot.update();

            telemetry.addLine("Front Left = 0, Front Right = 1, Back Left = 2, Back Right = 3");

            for(DcMotorEx motors : actualRobot.driveTrain.motors) {
                telemetry.addLine("Motor: " + motors.getPower());
                telemetry.addData("Encoders", motors.getCurrentPosition());
            }

            double imuValue = actualRobot.driveTrain.localizer.imu.getHeadingInRadians();
            telemetry.addData("IMU", Math.toDegrees(imuValue));
            telemetry.update();
        }



    }

}
