package org.firstinspires.ftc.teamcode.OpModes;

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

@TeleOp(name = "Actual TeleOP", group = "yes")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RobotBase actualRobot = new RobotBase(hardwareMap);

        actualRobot.initHardwareMap();
        actualRobot.driveTrain.setStartPosition(new Pose2D(0, 0, Math.toRadians(0)));

        waitForStart();

        while (opModeIsActive()) {
            double x = actualRobot.driverGamepad.leftJoystick.x();
            double y = actualRobot.driverGamepad.leftJoystick.y();
            double turn = actualRobot.driverGamepad.rightJoystick.x();

            /**
             *
             *  To switch between different Drive Settings have 3 of the
             *  actualRobot.driveTrain.[SOMETHING] Commented and leave one uncommented
             *
             *  ROBOT CENTRIC SETTINGS
             */

            actualRobot.driveTrain.setMotorPowers(x, y, turn);

            //actualRobot.driveTrain.setWeightedDrivePower(x, y, turn);

            /**
             *  FIELD CENTRIC SETTINGS
             */

            //actualRobot.driveTrain.driveFieldCentric(x, y, turn);

            //actualRobot.driveTrain.setFieldCentricDrivePower(x, y, turn);




            actualRobot.update();

            telemetry.addLine("Front Left = 0, Front Right = 1, Back Left = 2, Back Right = 3");

            for(DcMotorEx motors : actualRobot.driveTrain.motors) {
                telemetry.addLine("Motor: " + motors.getPower());
            }

            double imuValue = actualRobot.driveTrain.localizer.imu.getHeadingInRadians();
            telemetry.addData("IMU", Math.toDegrees(imuValue));
            telemetry.update();
        }



    }

}
