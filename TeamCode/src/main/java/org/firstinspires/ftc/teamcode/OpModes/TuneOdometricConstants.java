package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.TicksPerRev;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.TrackWidth;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.WheelRadius;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.middle;

@TeleOp(name = "Tune Robot Constants", group = "Tuning")
public class TuneOdometricConstants extends LinearOpMode {
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

        boolean ran = false;

        telemetry.addLine("Press X to start");
        telemetry.update();

        do {
            if(gamepad1.x || gamepad2.x) {

                ran = true;
            }
        } while(!ran);

        waitForStart();

        while(opModeIsActive()) {
            while(actualRobot.driveTrain.localizer.imu.getAccumulatedHeadingInDegrees() > 1000) {
                actualRobot.driveTrain.setMotorPowers(0, 0, 1);

                telemetry.addData("XYH", actualRobot.driveTrain.localizer.getPose().toString());
                telemetry.addData("Odom Angle", actualRobot.driveTrain.localizer.getPose().heading);
                telemetry.addData("IMU Angle", actualRobot.driveTrain.localizer.imu.getAccumulatedHeadingInDegrees());
                telemetry.addData("Latteral Encoder Reading", actualRobot.driveTrain.motors[3].getCurrentPosition());

                telemetry.addData("\nError", actualRobot.driveTrain.localizer.imu.getAccumulatedHeadingInDegrees()-actualRobot.driveTrain.localizer.accumulatedHeading);
                telemetry.addData("Error Percent", actualRobot.driveTrain.localizer.imu.getAccumulatedHeadingInDegrees()/actualRobot.driveTrain.localizer.accumulatedHeading);
                telemetry.addData("Track Width Multiplier", actualRobot.driveTrain.localizer.accumulatedHeading/actualRobot.driveTrain.localizer.imu.getAccumulatedHeadingInDegrees());
                telemetry.addData("New Track Width (L) ", TrackWidth * (actualRobot.driveTrain.localizer.accumulatedHeading/actualRobot.driveTrain.localizer.imu.getAccumulatedHeadingInDegrees()));

                double inchPerTick = (2.0 * Math.PI * WheelRadius / TicksPerRev);

                telemetry.addData("\nLatteral Encoder Offset", (actualRobot.driveTrain.motors[3].getCurrentPosition()/Math.toRadians(actualRobot.driveTrain.localizer.imu.getAccumulatedHeadingInDegrees()))*inchPerTick);
                telemetry.update();
            }
            actualRobot.driveTrain.stopDriveTrain();



            actualRobot.update();
        }

    }
}
