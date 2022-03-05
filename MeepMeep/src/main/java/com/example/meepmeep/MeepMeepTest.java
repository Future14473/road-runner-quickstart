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

//        double  startX = -35.5+48, startY = 70, startH = Math.toRadians(270),
//                preloadX = startX, preloadY = startY - 16, preloadH = 270,
//                preWharehouseX = 11, preWharehouseY = 72, preWahreHouseH = 0,
//                whareHouseX = 62, whareHouseY = 72, whareHouseH = 0;
        double preloadX = -29, preloadY = 49, preloadH = 270,
                startX = -35.5, startY = 70, startH = Math.toRadians(270),
                duckX = -54.5, duckY = 64.5, duckH = 182,
                preScoreDuckX = -37, preScoreDuckY = 65, preScoreDuckH = 290,
                scoreDuckX = -23, scoreDuckY = 50, scoreDuckH = 0,
                alignDuckTurn = -17,
                preParkX = 10, preParkY = 55, preParkH = 0,
                parkX = -69, parkY = 34, parkH = 0;
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType(DriveTrainType.TANK)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.5)
                .followTrajectorySequence(driveShim ->
                        driveShim.trajectorySequenceBuilder(new Pose2d(scoreDuckX, scoreDuckY, Math.toRadians(scoreDuckH)))
//                                .splineTo(new Vector2d(preloadX, preloadY), Math.toRadians(preloadH))
                                //                .splineTo(new Vector2d(preParkX, preParkY), Math.toRadians(preParkH))
                                .splineTo(new Vector2d(parkX, parkY), Math.toRadians(parkH))
//                                .turn(Math.toRadians(90))
                        .build()
                );
//        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
//                .setDriveTrainType(DriveTrainType.TANK)
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.5)
//                .followTrajectorySequence(driveShim ->
//                        driveShim.trajectorySequenceBuilder(new Pose2d(startX, startY, Math.toRadians(startH)))
////                                .splineTo(new Vector2d(duckX, duckY), Math.toRadians(duckH))
//                                .splineTo(new Vector2d(preScoreDuckX, preScoreDuckY), Math.toRadians(preScoreDuckH))
//                                .build()
//                );
        meepMeep.addEntity(bot)
//                .addEntity(bot2)
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .start();
    }
}