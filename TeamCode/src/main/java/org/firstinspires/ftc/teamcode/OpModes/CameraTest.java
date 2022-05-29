package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.vision.FreightFrenzyCamera;

@TeleOp(name = "Camera Test", group = "Testing")
public class CameraTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        FreightFrenzyCamera camera = new FreightFrenzyCamera(hardwareMap);

        telemetry.addLine("Waiting for Init");
        telemetry.update();

        camera.initWebCamera();
        camera.initPipeline();
        camera.startCamera();

        camera.startCameraStream(0);
        CameraStreamServer.getInstance().setSource(camera.getCamera());

        telemetry.clear();
        telemetry.addLine("Camera Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //camera.refreshScalars();

            telemetry.addData("Location", camera.sDeterminePosition());
            telemetry.update();
        }
    }
}
