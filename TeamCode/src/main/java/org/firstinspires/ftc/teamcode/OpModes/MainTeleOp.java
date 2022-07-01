package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;

@TeleOp(name = "Actual TeleOP", group = "yes")
public class MainTeleOp extends LinearOpMode {

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotBase actualRobot = new RobotBase(hardwareMap, gamepad1, gamepad2);

        actualRobot.initHardwareMap();
        actualRobot.driveTrain.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            /*
                DRIVE TRAIN
             */

            double x = actualRobot.driverGamepad.leftJoystick.x();
            double y = actualRobot.driverGamepad.leftJoystick.y();
            double turn = actualRobot.driverGamepad.rightJoystick.x();

            actualRobot.driveTrain.setWeightedDrivePower(new Pose2d(x, y, turn));

            /*
                ARM
             */

            if(actualRobot.operatorGamepad.circle()) {
                if(actualRobot.arm.getState() == Arm.ARM_STATE.PRIMED) {
                    actualRobot.arm.setState(Arm.ARM_STATE.DEPOSIT);
                }
            }

            if(actualRobot.operatorGamepad.triangle()) {
                actualRobot.arm.setState(Arm.ARM_STATE.PRIMED);
            }

            if(actualRobot.operatorGamepad.cross()) {
                actualRobot.arm.setState(Arm.ARM_STATE.RETURN);
            }

            if(actualRobot.operatorGamepad.leftTrigger.isPressed()) {
                if(actualRobot.arm.getState() == Arm.ARM_STATE.IDLE) {
                    actualRobot.arm.setState(Arm.ARM_STATE.INTAKING);
                }
                else {
                    actualRobot.arm.setState(Arm.ARM_STATE.IDLE);
                }
            }

            actualRobot.update();

            telemetry.addData("ARM STATE", actualRobot.arm.getState().toString());

            telemetry.addLine("Front Left = 0, Back Left = 1, Back Right = 2, Back Left = 3");

            for(DcMotorEx motors : actualRobot.driveTrain.motors) {
                //telemetry.addLine("Motor: " + motors.getPower());
                telemetry.addData("Encoders", motors.getCurrentPosition());
            }

            telemetry.addData("Pos", actualRobot.driveTrain.getPoseEstimate().toString());
            telemetry.update();
        }



    }

}
