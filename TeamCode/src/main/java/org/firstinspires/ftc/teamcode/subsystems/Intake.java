package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

// idk if this is right, i found it in TeleOpDrive
public class Intake {

    private static CRServo leftServo, rightServo;

    public static void register(HardwareMap hardware) {
        Hardware.initServos(hardware);

        leftServo = Hardware.getLeftServo();
        rightServo = Hardware.getRightServo();
    }

    public static void setIntakeMode(String mode)
    {
        if(mode.equals("intake"))
        {
            leftServo.setDirection(CRServo.Direction.REVERSE);
            rightServo.setDirection(CRServo.Direction.FORWARD);
        }
        else if(mode.equals("outtake"))
        {
            leftServo.setDirection(CRServo.Direction.FORWARD);
            rightServo.setDirection(CRServo.Direction.REVERSE);
        }

        if(leftServo.getPower() != 0)
        {
            leftServo.setPower(0);
            leftServo.setPower(0.7);
        }
        if(rightServo.getPower() != 0)
        {
            rightServo.setPower(0);
            rightServo.setPower(0.7);
        }
    }
}
