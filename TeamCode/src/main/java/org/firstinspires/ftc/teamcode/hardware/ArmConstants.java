package org.firstinspires.ftc.teamcode.hardware;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
public class ArmConstants {
    public static class Turret {
        double ticksPerRev = 145.1;

        double panKitRes = 28;

        double outputResolution = ticksPerRev * panKitRes;

        double halfRotation = outputResolution / 2;
    }

    public static class Orienter {
        public static double flat = 0.49;
        public static double deposit = 0.6;
        public static double up = 0.35;
        public static double zero = 0.82;
        public static double goofy = 0.2;
    }

    public static class Lift {
        public static double startPosition = 31;
        public static double floating = 50;
        public static double score = 150;
        public static double goofy = 190;

        // Lift Constants
        public static int ticksPerRev = 3896;
        public static double ticksPerDegree = ticksPerRev / 360.0;

        // Cos PID stuff
        public static double kP = 0.015, kI = 0, kD = 0;
        public static double ffCoef = 0.3;
        public static PIDCoefficients liftCoef = new PIDCoefficients(kP, kI, kD);
        public static BasicPID liftPID = new BasicPID(liftCoef);
    }

    public static class Intake {
        public static double intakeSpeed = 1;
        public static double outtakeSpeed = 0.4;
    }

    public static class Flap {
        public static double open = 0;
        public static double closed = 0.3;
    }

    public static class FreightDetector {
        public static double distanceTolerance = 50;

        public static int lR = 0, lG = 0, lB = 0, lA = 0; // Lower
        public static int uR = 0, uG = 0, uB = 0, uA = 0; // Upper
    }
}
