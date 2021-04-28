package org.firstinspires.ftc.teamcode.ourOpModes.test;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Pathing;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;

import java.util.Scanner;

public class TurningPID {
    public static void main(String[] args) {
        Scanner scan = new Scanner(System.in);

        while (true){
            System.out.println("Enter target dir");
            double dest_heading = Math.toRadians(scan.nextInt());
            double power = turn_to_heading_PID(0, dest_heading, 1);
            System.out.println("Power: " + power);
        }
    }

    static PIDFController turn_PID = new PIDFController(SampleMecanumDrive.HEADING_PID);
    static double turn_to_heading_PID(double curr_heading, double dest_heading, double power_coeff){

        double heading_delta = RotationUtil.turnLeftOrRight(
                curr_heading, dest_heading, Math.PI/2);

        turn_PID.setTargetPosition(heading_delta);
        turn_PID.setTargetVelocity(0);

         return turn_PID.update(0) * power_coeff;

    }
}
