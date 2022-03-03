package com.example.meepmeep;

import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {

        double  startX = -35.5+48, startY = 70, startH = Math.toRadians(270),
                preloadX = startX, preloadY = startY - 16, preloadH = 270,
                preWharehouseX = 11, preWharehouseY = 61, preWahreHouseH = 0,
                whareHouseX = 62, whareHouseY = 62, whareHouseH = 0;
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType(DriveTrainType.TANK)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(driveShim ->
                        driveShim.trajectorySequenceBuilder(new Pose2d(preloadX, preloadY, Math.toRadians(preloadH)))
//                                .splineTo(new Vector2d(preScoreDuckX, preScoreDuckY), Math.toRadians(preScoreDuckH))
//                                .splineTo(new Vector2d(scoreDuckX, scoreDuckY), Math.toRadians(scoreDuckH))
//                        .splineTo(new Vector2d(37, 65), Math.toRadians(180))
//                                .splineTo(new Vector2d(5, 65),Math.toRadians(0))
//                                .splineTo(new Vector2d(-4, 59), Math.toRadians(0))
//                                .splineTo(new Vector2d(-4, 43),Math.toRadians(240))
                                .splineTo(new Vector2d(preWharehouseX, preWharehouseY), Math.toRadians(preWahreHouseH))
                                .splineTo(new Vector2d(whareHouseX, whareHouseY), Math.toRadians(whareHouseH))
                        .build()
                );
        meepMeep.addEntity(bot)
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .start();
    }
}