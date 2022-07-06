package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Duck.*;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gamepad.Trigger;
import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Duck;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

@Config
@TeleOp(name = "Actual TeleOP", group = "yes")
public class MainTeleOp extends LinearOpMode {
    public static double liftSpeed = -0.5;
    public static double turretSpeed = 0.5;

    public static double slowTurret = 0.25;
    public static double fastTurret = 0.5;

    public static double slowLift = -0.25;
    public static double fastLift = -0.5;

    boolean reset = false;

    Timer fsmTimer = new Timer();

    public static double or = 0.2;
    public static double flap = 0.3;

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotBase actualRobot = new RobotBase(hardwareMap, gamepad1, gamepad2);


        actualRobot.initHardwareMap();
        actualRobot.driveTrain.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            //actualRobot.update();

            /*
                DRIVE TRAIN
             */

            double x = actualRobot.driverGamepad.leftJoystick.x();
            double y = actualRobot.driverGamepad.leftJoystick.y();
            double turn = actualRobot.driverGamepad.rightJoystick.x();

            actualRobot.driveTrain.setWeightedDrivePower(new Pose2d(-y, -x, -turn));

            if(actualRobot.operatorGamepad.leftTrigger.howPressed() == Trigger.PRESS.lightly) {
                actualRobot.arm.openFlap();
            }
            else if(actualRobot.operatorGamepad.leftTrigger.howPressed() == Trigger.PRESS.down) {
                actualRobot.arm.intake.setPower(actualRobot.operatorGamepad.rightTrigger.value()/2.5);
                actualRobot.arm.openFlap();
            }
            else {
                actualRobot.arm.closeFlap();
            }

            if(gamepad1.circle) {
                actualRobot.duck.setVelocity(-600);
            }
            else {
                actualRobot.duck.setVelocity(0);
            }



            if(gamepad2.left_bumper) {
                turretSpeed = slowTurret;
                liftSpeed = slowLift;
            }
            if(gamepad2.right_bumper) {
                turretSpeed = fastTurret;
                liftSpeed = fastLift;
            }

            if(gamepad2.dpad_up) {
                or -= 0.005;
            }
            if(gamepad2.dpad_down) {
                or += 0.005;
            }

            if(gamepad2.dpad_left) {
                or = ArmConstants.Orienter.intake;
            }
            if(gamepad2.dpad_right) {
                or = ArmConstants.Orienter.up;
            }

            actualRobot.arm.setIntakeOrientation(or);
            actualRobot.arm.turret.setPower(gamepad2.left_stick_x * turretSpeed);
            actualRobot.arm.setAdjustedPower(gamepad2.right_stick_y * liftSpeed);
            actualRobot.arm.intake.setPower(-actualRobot.operatorGamepad.rightTrigger.value());

            telemetry.addData("Or", or);
            telemetry.addData("Duck Velo", actualRobot.duck.getVelocity());
            telemetry.addData("Turrret Pos", actualRobot.arm.turret.getCurrentPosition());
            telemetry.addData("lift speed", liftSpeed);
            telemetry.addData("turret speed", turretSpeed);
            telemetry.addData("F Distance", actualRobot.arm.detector.getDistance(DistanceUnit.MM));
            telemetry.addData("Has Freight", actualRobot.arm.hasFreight());
            telemetry.update();
        }
    }

}
