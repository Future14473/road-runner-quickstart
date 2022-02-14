package com.example.meepmeeptest

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveTrainType
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

fun carouselPath(blue: Boolean, meepMeep: MeepMeep): RoadRunnerBotEntity {
    return DefaultBotBuilder(meepMeep)
            .setDriveTrainType(DriveTrainType.TANK)
            .setDimensions(16.877953, 16.1417)
            .setConstraints(90.0, 90.0, Math.toRadians(774.5043079608481), Math.toRadians(774.5043079608481), 14.42126)
            .followTrajectorySequence {drive  ->
                val trajectoryBuilder =
                        drive.trajectorySequenceBuilder(Pose2d(0.0,0.0, Math.toRadians(90.0)))
                                .setReversed(true)
                trajectoryBuilder
                        .setReversed(false)
                        .setReversed(true)
                        .setReversed(false)
                        .build()
            }
}
