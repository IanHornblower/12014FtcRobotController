package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

@TeleOp(name = "Motor Tester", group = "yes")
public class MotorTesting extends LinearOpMode {

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotBase actualRobot = new RobotBase(hardwareMap, gamepad1, gamepad2);

        actualRobot.initHardwareMap();
        actualRobot.driveTrain.setPoseEstimate(new Pose2d(0, 0, 0));

        double fl = 0, bl = 0, fr = 0, br = 0;

        waitForStart();

        while (opModeIsActive()) {
            /*
                DRIVE TRAIN
             */

            double x = actualRobot.driverGamepad.leftJoystick.x();
            double y = actualRobot.driverGamepad.leftJoystick.y();
            double turn = actualRobot.driverGamepad.rightJoystick.x();

            actualRobot.driveTrain.setWeightedDrivePower(new Pose2d(x, y, turn));



            if(gamepad1.y) {
                fl = 1;
            }
            else {
                fl = 0;
            }

            if(gamepad1.x) {
                bl = 1;
            }
            else {
                bl = 0;
            }

            if(gamepad1.b) {
                fr = 1;
            }
            else {
                fr = 0;
            }

            if(gamepad1.a) {
                br = 1;
            }
            else {
                br = 0;
            }

            actualRobot.driveTrain.setMotorPowers(fl, bl, br, fr);

            actualRobot.update();

            telemetry.addLine("Front Left = 0, Front Right = 1, Back Left = 2, Back Right = 3");

            for(DcMotorEx motors : actualRobot.driveTrain.motors) {
                telemetry.addLine("Motor: " + motors.getPower());
                //telemetry.addData("Encoders", motors.getCurrentPosition());
            }

            telemetry.addData("Pos", actualRobot.driveTrain.getPoseEstimate().toString());
            telemetry.update();
        }



    }

}
