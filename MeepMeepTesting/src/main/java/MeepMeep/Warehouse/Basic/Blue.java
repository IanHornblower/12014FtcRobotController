package MeepMeep.Warehouse.Basic;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import static MeepMeep.AutoConstants.*;
import static MeepMeep.Constants.*;

public class Blue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        Pose2d START = warehouseBlueStart;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(width, height)
                .setConstraints(MAX_VELO, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(START)
                                .lineToLinearHeading(warehouseBlueAllianceHub)
                                .splineTo(outerWarehouseBlue.vec(), outerWarehouseBlue.getHeading())
                                .lineToLinearHeading(innerWarehouseBlue)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_CRI_SHARED_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}