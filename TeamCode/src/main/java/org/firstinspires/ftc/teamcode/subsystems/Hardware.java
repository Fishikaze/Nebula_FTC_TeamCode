package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware
{


    private DcMotor frontRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor linearActuator;
    private DcMotor armSlideMotor;
    private DcMotor armRotationMotor;
    private CRServo leftServo;
    private CRServo rightServo;
    public void hardwareMapDrivetrain(HardwareMap hardwareMap)
    {
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
    }

    public void hardwareMapServos(HardwareMap hardwareMap)
    {
        leftServo = hardwareMap.get(CRServo.class,"ls");
        rightServo = hardwareMap.get(CRServo.class,"rs");
    }
    public void hardwareMapLinearActuator(HardwareMap hardwareMap)
    {
        linearActuator = hardwareMap.get(DcMotor.class, "la1");
    }
    public void hardwareMapArm(HardwareMap hardwareMap)
    {
        armSlideMotor = hardwareMap.get(DcMotor.class, "asm");
        armRotationMotor = hardwareMap.get(DcMotor.class, "arm");
    }
    public DcMotor getFrontLeftDrive()
    {
        return frontLeftDrive;
    }
    public DcMotor getBackLeftDrive()
    {
        return backLeftDrive;
    }
    public DcMotor getFrontRightDrive()
    {
        return frontRightDrive;
    }
    public DcMotor getBackRightDrive()
    {
        return backRightDrive;
    }
    public DcMotor getLinearActuator() { return linearActuator;}
    public DcMotor getArmSlideMotor() { return  armSlideMotor;}
    public DcMotor getArmRotationMotor() { return  armRotationMotor;}
    public CRServo getLeftServo()
    {
        return leftServo;
    }
    public CRServo getRightServo()
    {
        return rightServo;
    }
}
