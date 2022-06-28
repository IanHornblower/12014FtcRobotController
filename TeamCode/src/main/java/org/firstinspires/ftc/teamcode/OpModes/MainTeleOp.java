package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.middle;

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

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotBase actualRobot = new RobotBase(hardwareMap, gamepad1, gamepad2);

        actualRobot.initHardwareMap();
        actualRobot.driveTrain.setStartPosition(new Pose2D(0, 0, Math.toRadians(0)));

        actualRobot.arm.setStartPosition(middle, 0);
        actualRobot.arm.setManualValue(()-> actualRobot.operatorGamepad.leftJoystick.x());

        actualRobot.arm.setState(Arm.ARM_STATE.IDLE);

        actualRobot.arm.update();

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

            actualRobot.arm.setIntakePower(actualRobot.operatorGamepad.leftTrigger.value()/5);

            if(actualRobot.operatorGamepad.circle()) {
                actualRobot.arm.setState(Arm.ARM_STATE.DEPOSIT);
            }

            if(actualRobot.operatorGamepad.triangle()) {
                actualRobot.arm.setState(Arm.ARM_STATE.PRIMED);
            }

            if(actualRobot.operatorGamepad.cross()) {
                actualRobot.arm.setState(Arm.ARM_STATE.RETURN);
            }

            if(actualRobot.operatorGamepad.leftTrigger.isPressed()) {
                actualRobot.arm.setState(Arm.ARM_STATE.INTAKING);
            }
            else {
                actualRobot.arm.setState(Arm.ARM_STATE.IDLE);
            }


            //switch (actualRobot.operatorGamepad.dpad()) {
            //    case up:
            //        armPos = 190;
            //        orienterPos = 0.2;
            //        break;
            //    case down:
            //        armPos = 35;
            //        orienterPos = 0.4;
            //        armTime.reset();
            //        if(armTime.currentSeconds() > 0.5) {
            //            armPos = 35;
            //        }
            //        break;
            //    case left:
            //        orienterPos = 0.49;
            //    default:
//
            //        break;
            //}

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
