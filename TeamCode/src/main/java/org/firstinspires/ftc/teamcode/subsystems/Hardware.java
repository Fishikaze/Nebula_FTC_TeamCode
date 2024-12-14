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
    private DcMotor linearActuator1;
    private DcMotor linearActuator2;

    private CRServo leftServo;
    private CRServo rightServo;
    public void hardwareMapMotors(HardwareMap hardwareMap)
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
        linearActuator1 = hardwareMap.get(DcMotor.class, "la1");
        linearActuator2 = hardwareMap.get(DcMotor.class, "la2");
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
    public DcMotor getLinearActuator1(){ return linearActuator1;}
    public DcMotor getLinearActuator2(){ return linearActuator2;}
    public CRServo getLeftServo()
    {
        return leftServo;
    }
    public CRServo getRightServo()
    {
        return rightServo;
    }
}
