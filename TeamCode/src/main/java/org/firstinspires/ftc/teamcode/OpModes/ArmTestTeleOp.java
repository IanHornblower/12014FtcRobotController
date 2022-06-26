package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.math.Pose2D;

@Config
@TeleOp(name = "Arm Test", group = "yes")
public class ArmTestTeleOp extends LinearOpMode {

    public static double or = 0.49;
    public static double deg = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotBase actualRobot = new RobotBase(hardwareMap, gamepad1, gamepad2);

        actualRobot.initHardwareMap();
        actualRobot.driveTrain.setStartPosition(new Pose2D(0, 0, Math.toRadians(0)));

        actualRobot.arm.setStartPosition(middle, 0);
        actualRobot.arm.setManualValue(()-> actualRobot.driverGamepad.leftJoystick.x());

        waitForStart();

        while (opModeIsActive()) {

            actualRobot.arm.setLiftPosition(deg);

            actualRobot.arm.setIntakeOrientation(or);

            actualRobot.arm.setIntakePower(actualRobot.driverGamepad.rightTrigger.value()-actualRobot.driverGamepad.leftTrigger.value());

            actualRobot.update();

            telemetry.addData("Has Freight", actualRobot.arm.hasFreight());
            telemetry.addData("Is Cube", actualRobot.arm.hasCube());

            telemetry.addData("Color Sensor Distance", actualRobot.arm.detector.getDistance(DistanceUnit.MM));
            telemetry.addData("Color Sensor Color", actualRobot.arm.detector.getNormalizedColors().toString());

            telemetry.addData("Arm", actualRobot.driverGamepad.rightJoystick.y());
            telemetry.addData("Arm Set Value", actualRobot.arm.armPosition);
            telemetry.addData("Arm State", actualRobot.arm.armState.toString());

            telemetry.addData("Arm Lift Orentation", actualRobot.arm.getOrientation());
            telemetry.addData("Arm Position", actualRobot.arm.turretLift.getCurrentPosition());

            double imuValue = actualRobot.driveTrain.localizer.imu.getHeadingInRadians();
            telemetry.addData("IMU", Math.toDegrees(imuValue));
            telemetry.update();
        }



    }

}
