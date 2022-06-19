package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.asyncUtil.ActionSequence;
import org.firstinspires.ftc.teamcode.asyncUtil.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.asyncUtil.actions.*;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IMU;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

@TeleOp(name = "Action Testing", group = "no")
public class ActionTeleOpTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotBase actualRobot = new RobotBase(hardwareMap, gamepad1, gamepad2);
        ActionSequenceRunner runner = new ActionSequenceRunner(actualRobot);

        actualRobot.initHardwareMap();
        //actualRobot.driveTrain.setStartPosition(new Pose2D(63, 12, Math.toRadians(90)));
        actualRobot.driveTrain.setStartPosition(new Pose2D(0, 0, Math.toRadians(0)));

        ActionSequence as = new ActionSequence();
        as.addAction(new IMURotate(actualRobot.driveTrain, Math.toRadians(90)));
        as.addAction(new Wait(1));
        as.addAction(new IMURotate(actualRobot.driveTrain, Math.toRadians(0)));
        /*
         *  as.addAction(new EncoderDrive(actualRobot.driveTrain, 0.5, 0.5, 0, 5000));
         *  as.addAction(new Wait(.5));
         *  as.addAction(new IMURotate(actualRobot.driveTrain, Math.toRadians(90)));
         *  as.addAction(new Wait(.5));
         *  as.addAction(new EncoderDriveEx(actualRobot.driveTrain, 1, 1, Math.toRadians(0), 5000));
         */

        runner.setActionSequence(as.getActionList());

        waitForStart();

        while (opModeIsActive()) {

            // it works :)

            if(!runner.isComplete()) {
                runner.update();
            }
            else {
                stop();
            }

            telemetry.addLine("Front Left = 0, Front Right = 1, Back Left = 2, Back Right = 3");

            for(DcMotorEx motors : actualRobot.driveTrain.motors) {
                telemetry.addLine("Motor: " + motors.getPower());
            }

            for(DcMotorEx motors : actualRobot.driveTrain.motors) {
                telemetry.addData("Motor Encoder Position", motors.getCurrentPosition());
            }

            double imuValue = actualRobot.driveTrain.localizer.imu.getHeadingInRadians();

            telemetry.addData("IMU", Math.toDegrees(imuValue));
            telemetry.addData("Kalman Heading", Math.toDegrees(actualRobot.driveTrain.localizer.getKalmanHeading()));
            telemetry.addData("Pose Estimate", actualRobot.driveTrain.localizer.getPose().toString());
            telemetry.update();
        }



    }

}
