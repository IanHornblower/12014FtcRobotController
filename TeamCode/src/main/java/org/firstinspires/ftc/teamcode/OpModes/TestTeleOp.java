package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.control.*;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;

/**
 *
 *
 * DO NOT RUN THIS AT ALL IT WILL SHIT IT SELF
 *
 *
 *
 */

@Disabled
@TeleOp(name = "Ogga Booga TestTeleoP", group = "no")
public class TestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotBase actualRobot = new RobotBase(hardwareMap,gamepad1, gamepad2);
        TrajectoryFollower f = new TrajectoryFollower(actualRobot.driveTrain, 4.5, 0.5);

        actualRobot.initHardwareMap();
        actualRobot.driveTrain.setStartPosition(new Pose2D(0, 0, Math.toRadians(0)));

        Trajectory joe = new Trajectory();

        joe.add(new Pose2D(0, 0, 0));
        joe.add(new Pose2D(24, 24, Math.toRadians(90)));
        joe.add(new Pose2D(24, 48, 0));
        joe.add(new Pose2D(24, 60, 0));

        Trajectory oej = new Trajectory();

        oej.add(new Pose2D(24, 60, 0));
        oej.add(new Pose2D(24, 48, 0));
        oej.add(new Pose2D(24, 24, Math.toRadians(90)));
        oej.add(new Pose2D(0, 0, 0));

        ArrayList<Trajectory> s = new ArrayList<>();
        s.add(joe);
        s.add(oej);

        TrajectorySequence trajS = new TrajectorySequence(s);

        int count = 0;
        boolean start = false;

        waitForStart();

        while (opModeIsActive()) {
            double x = actualRobot.driverGamepad.leftJoystick.x();
            double y = actualRobot.driverGamepad.leftJoystick.y();
            double turn = actualRobot.driverGamepad.rightJoystick.x();

            // TODO: get a better Traj Seq system


            if(!start) {
                f.setTrajectory(joe);
                start = true;
            }
            if(!oej.isStarted() && f.isCompleted()) {
                f.reverse();
                f.setTrajectory(oej);
            }
            if(joe.isComplete() && oej.isComplete()) {
                f.setState(TrajectoryFollower.STATE.STOPPED);
            }
            if(!joe.isComplete() || !oej.isComplete()) {
                f.AnglePursuitFollower(10);
            }
            if(oej.isComplete()) {
                actualRobot.driveTrain.rotate(0);
            }
            actualRobot.update();






























            telemetry.addLine("Front Left = 0, Front Right = 1, Back Left = 2, Back Right = 3");

            for(DcMotorEx motors : actualRobot.driveTrain.motors) {
                telemetry.addLine("Motor: " + motors.getPower());
            }

            double imuValue = actualRobot.driveTrain.localizer.imu.getHeadingInRadians();

            telemetry.addData("IMU", Math.toDegrees(imuValue));
            telemetry.addData("Kalman Heading", Math.toDegrees(actualRobot.driveTrain.localizer.getKalmanHeading()));
            telemetry.addData("Pose Estimate", actualRobot.driveTrain.localizer.getPose().toString());
            telemetry.update();
        }



    }

}
