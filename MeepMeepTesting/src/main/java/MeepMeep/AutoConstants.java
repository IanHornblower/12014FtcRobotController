package MeepMeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AutoConstants {

    // Starts
    public static Pose2d warehouseRedStart = new Pose2d(11.5, -63, Math.toRadians(270));
    public static Pose2d warehouseBlueStart = new Pose2d(11.5, 63, Math.toRadians(90));

    public static Pose2d middleRedStart = new Pose2d(11.5, -64, Math.toRadians(0));
    public static Pose2d middleBlueStart = new Pose2d(11.5, 64, Math.toRadians(0));

    public static Pose2d coopRedStart = new Pose2d(11.5, -64, Math.toRadians(0));
    public static Pose2d coopBlueStart = new Pose2d(11.5, 64, Math.toRadians(0));

    // First hub Location
    public static Pose2d warehouseRedAllianceHub = new Pose2d(-3, -42, Math.toRadians(300));
    public static Pose2d warehouseBlueAllianceHub = new Pose2d(-3, 42, Math.toRadians(60));

    // OuterWarehouse Location
    public static Pose2d outerWarehouseRed = new Pose2d(25, -64, Math.toRadians(0));
    public static Pose2d outerWarehouseBlue = new Pose2d(25, 64, Math.toRadians(0));

    // InnerWarehouse
    public static Pose2d innerWarehouseRed = new Pose2d(37, -64, Math.toRadians(0));
    public static Pose2d innerWarehouseBlue = new Pose2d(37, 64, Math.toRadians(0));



}
