package org.firstinspires.ftc.teamcode.roadrunnerext

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d

fun Pose2d.toRRPose2d(): com.acmerobotics.roadrunner.geometry.Pose2d {
    return com.acmerobotics.roadrunner.geometry.Pose2d(this.x, this.y, this.heading)
}

fun com.acmerobotics.roadrunner.geometry.Pose2d.flipHeading(): com.acmerobotics.roadrunner.geometry.Pose2d {
    return com.acmerobotics.roadrunner.geometry.Pose2d(this.x, this.y, this.heading + Math.PI)
}


 fun com.acmerobotics.roadrunner.geometry.Pose2d.toInches(): com.acmerobotics.roadrunner.geometry.Pose2d {
     return com.acmerobotics.roadrunner.geometry.Pose2d(
         this.x * 39.3701,
         this.y * 39.701,
         this.heading
     )
 }

fun Pose2d.toMeters(): Pose2d {
    return Pose2d(this.x / 39.3701, this.y / 39.701, this.heading)
}

fun Double.flip(negative: Boolean): Double {
    return if (negative) -this else this
}

fun com.acmerobotics.roadrunner.geometry.Pose2d.flip(negative: Boolean): com.acmerobotics.roadrunner.geometry.Pose2d {
    return if (negative) com.acmerobotics.roadrunner.geometry.Pose2d(this.x, -this.y, -this.heading) else this
}

fun Vector2d.flip(negative: Boolean): Vector2d {
    return if (negative) Vector2d(this.x, -this.y) else this
}