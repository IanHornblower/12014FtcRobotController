package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.math.Pose2D;

@TeleOp(name = "Arm Test", group = "yes")
public class ArmTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        RobotBase actualRobot = new RobotBase(hardwareMap, gamepad1, gamepad2);

        actualRobot.initHardwareMap();
        actualRobot.driveTrain.setStartPosition(new Pose2D(0, 0, Math.toRadians(0)));

        actualRobot.arm.setStartPosition(middle, 0);
        actualRobot.arm.setManualValue(()-> actualRobot.driverGamepad.leftJoystick.x());

        waitForStart();

        while (opModeIsActive()) {

            actualRobot.arm.turretLift.setPower(actualRobot.driverGamepad.rightJoystick.y());

            actualRobot.update();

            telemetry.addData("Arm", actualRobot.driverGamepad.rightJoystick.y());
            telemetry.addData("Arm Set Value", actualRobot.arm.armPosition);
            telemetry.addData("Arm State", actualRobot.arm.armState.toString());

            double imuValue = actualRobot.driveTrain.localizer.imu.getHeadingInRadians();
            telemetry.addData("IMU", Math.toDegrees(imuValue));
            telemetry.update();
        }



    }

}
