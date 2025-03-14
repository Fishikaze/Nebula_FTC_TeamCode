package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Gripper {


    public static Servo vs;
    public static Servo hs;


    public static void register(HardwareMap hardware) {
        Hardware.initServos(hardware);


        vs = Hardware.getVerticalServo();
        hs = Hardware.getHorizontalServo();
    }


    //TEST ASAP
    public static void toggleHorizontal() {
        double pos = hs.getPosition();
        hs.setPosition(pos >= 0.6 ? 0.2 : 0.8);
    }


    //TEST ASAP
    public static void toggleVertical() {
        vs.setPosition(.75);
    }


    public static void move(boolean up) {
        double pos = vs.getPosition();
        vs.setPosition(up ? pos- 0.05 : pos + 0.05);
    }
}
