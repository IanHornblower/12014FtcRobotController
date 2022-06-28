package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.GamepadE;
import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

@TeleOp(name = "GamepadEx Testing", group = "Testing")
public class GamepadJoystick extends LinearOpMode {
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Waiting for Init");
        telemetry.update();

        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        gamepadEx.cross.setToggle();

        waitForStart();

        while(opModeIsActive()) {
            gamepadEx.update();

            telemetry.addLine(gamepadEx.toString("Gamepad 1"));
            telemetry.addData("Cross", gamepadEx.cross());
            telemetry.addData("Circle", gamepadEx.circle());
            telemetry.update();
        }
    }
}
