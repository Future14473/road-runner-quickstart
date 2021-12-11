package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RetractableOdo {
    Servo perpendicular, parallel;
    // for down position parallel it can go into the floor a little bit because of springing!
    public static double upPositionPerp = 0.3, downPositionPerp = 0.17, upPositionParallel = 0.28, downPositionParallel = 0.09;

    public RetractableOdo(HardwareMap hardwareMap){
        perpendicular = hardwareMap.get(Servo.class, "perpendicularOdo");
        parallel = hardwareMap.get(Servo.class, "parallelOdo");
        parallel.setDirection(Servo.Direction.REVERSE);
    }

    public void upOdo(){
        perpendicular.setPosition(upPositionPerp);
        parallel.setPosition(upPositionParallel);
    }

    public void downOdo(){
        perpendicular.setPosition(downPositionPerp);
        parallel.setPosition(downPositionParallel);
    }

    public double getPerpPos(){return perpendicular.getPosition();}
    public double getParallelPos(){return parallel.getPosition();}
}
