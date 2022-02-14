package com.example.meepmeeptest

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity
import javax.sound.sampled.Line

fun MeepMeep.addMultiPath(botEntityBuilder: (Boolean, MeepMeep) -> RoadRunnerBotEntity): MeepMeep {
    return this
            .addEntity(botEntityBuilder(true, this))
            .addEntity(botEntityBuilder(false, this))
}
fun Double.flip(negative: Boolean): Double {
    return if (negative) -this else this
}

fun Pose2d.flip(negative: Boolean): Pose2d {
    return if (negative) Pose2d(this.x, -this.y, -this.heading) else this
}

fun Vector2d.flip(negative: Boolean): Vector2d {
    return if (negative) Vector2d(this.x, -this.y) else this
}