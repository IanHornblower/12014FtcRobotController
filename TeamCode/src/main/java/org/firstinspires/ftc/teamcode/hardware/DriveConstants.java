package org.firstinspires.ftc.teamcode.hardware;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

public class DriveConstants {

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double TURN_WEIGHT = 3;

    public static PIDCoefficients xPID = new PIDCoefficients(0.1, 0 ,0);
    public static PIDCoefficients yPID = new PIDCoefficients(0.1, 0 ,0);
    public static PIDCoefficients headingPID = new PIDCoefficients(3, 0 ,0);

    public static PIDCoefficients forwardPID = new PIDCoefficients(0.08, 0 ,0);
    public static PIDCoefficients turnPID = new PIDCoefficients(3, 0 ,0);

}
