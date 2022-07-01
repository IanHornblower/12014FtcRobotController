package MeepMeep.Warehouse.Basic;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import static MeepMeep.AutoConstants.innerWarehouseRed;
import static MeepMeep.AutoConstants.outerWarehouseRed;
import static MeepMeep.AutoConstants.warehouseRedAllianceHub;
import static MeepMeep.AutoConstants.warehouseRedStart;
import static MeepMeep.Constants.MAX_ACCEL;
import static MeepMeep.Constants.MAX_ANG_ACCEL;
import static MeepMeep.Constants.MAX_ANG_VEL;
import static MeepMeep.Constants.MAX_VELO;
import static MeepMeep.Constants.height;
import static MeepMeep.Constants.trackWidth;
import static MeepMeep.Constants.width;

public class Red {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        Pose2d START = warehouseRedStart;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(width, height)
                .setConstraints(MAX_VELO, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(START)
                                .lineToLinearHeading(warehouseRedAllianceHub)
                                .splineTo(outerWarehouseRed.vec(), outerWarehouseRed.getHeading())
                                .lineToLinearHeading(innerWarehouseRed)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_CRI_SHARED_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}