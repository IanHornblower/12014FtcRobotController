package org.firstinspires.ftc.teamcode.hardware;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.InterpLUT;

public class ArmConstants {
    @Config
    public static class Turret {

        public static double ticksPerRev = 145.1;

        public static double panKitRes = 28;

        public static double outputResolution = ticksPerRev * panKitRes;

        public static double halfRotation = outputResolution / 2;
        public static double startPosition = 180;
        public static double kP = 0.0, kI = 0, kD = 0;
        public static PIDCoefficients turretPIDCoef = new PIDCoefficients(kP, kI, kD);
        public static BasicPID turretPID = new BasicPID(turretPIDCoef);

        public static double simpleTolerance = 100;

        public static InterpLUT turretLUT = new InterpLUT();
        public static void initTurretLUT() {
            turretLUT.add(0, 0);
            turretLUT.add(180, halfRotation);
            turretLUT.add(360, outputResolution);
        }
    }

    @Config
    public static class Orienter {
        public static double flat = 0.49;
        public static double deposit = 0.6;
        public static double up = 0.35;
        public static double zero = 0.82;
        public static double goofy = 0.2;
        public static double start = flat;
    }

    @Config
    public static class Lift {
        public static double startPosition = 31;
        public static double floating = 50;
        public static double score = 150;
        public static double goofy = 190;

        // Lift Constants
        public static int ticksPerRev = 3896;
        public static double ticksPerDegree = ticksPerRev / 360.0;

        // Cos PID stuff
        public static double kP = 0.0, kI = 0, kD = 0;
        public static double ffCoef = 0.0;
        public static PIDCoefficients liftCoef = new PIDCoefficients(kP, kI, kD);
        public static BasicPID liftPID = new BasicPID(liftCoef);
    }

    @Config
    public static class Intake {
        public static double intakeSpeed = 1;
        public static double outtakeSpeed = 0.4;
    }

    @Config
    public static class Flap {
        public static double open = 0;
        public static double closed = 0.3;
    }

    @Config
    public static class FreightDetector {
        public static double distanceTolerance = 50;

        public static int lR = 0, lG = 0, lB = 0, lA = 0; // Lower
        public static int uR = 0, uG = 0, uB = 0, uA = 0; // Upper
    }
}
