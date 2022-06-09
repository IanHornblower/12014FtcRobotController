package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

        actualRobot.arm.setManualValue(()-> actualRobot.driverGamepad.leftJoystick.x());

        waitForStart();

        // TODO: -> Test Lift -> Make sure Intake Flap Works -> Integrate intake flap & intake -> mix em' all

        while (opModeIsActive()) {

            actualRobot.arm.turretLift.setPower(actualRobot.driverGamepad.rightJoystick.y());

            if(actualRobot.driverGamepad.left_bumper()) {
                actualRobot.arm.setState(Arm.ARM_STATE.manualPointControl);
            }
            else if(actualRobot.driverGamepad.right_bumper()) {
                actualRobot.arm.setState(Arm.ARM_STATE.manual);
            }

            switch (actualRobot.driverGamepad.dpad()) {
                case up:
                    actualRobot.arm.setPointArmPosition(middle);
                    break;
                case left:
                    actualRobot.arm.setPointArmPosition(leftBound);
                    break;
                case right:
                    actualRobot.arm.setPointArmPosition(rightBound);
                    break;
                default:
                    break;
            }

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
