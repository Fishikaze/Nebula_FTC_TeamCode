package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm
{
    public DcMotor armSlideMotor;
    public DcMotor armRotateMotor;
    private Telemetry telemetry;
    private final int ARM_TOP_POSITION = 10;
    private final int ARM_BOTTOM_POSITION = 0;
    private final int ARM_EXTENDED_POSITION = 10;
    private final int ARM_RETRACTED_POSITION = 0;
    public Arm (HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        Hardware hardware = new Hardware();
        hardware.hardwareMapArm(hardwareMap);
        armSlideMotor = hardware.getArmSlideMotor();
        armRotateMotor = hardware.getArmRotationMotor();
    }
    public void setArmLevel(String pos)
    {
        if( pos.equals("TOP"))
        {
            armRotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armRotateMotor.setPower(0.7);
            armRotateMotor.setTargetPosition(ARM_TOP_POSITION);
            armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (pos.equals("BOTTOM"))
        {
            armRotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armRotateMotor.setPower(-0.7);
            armRotateMotor.setTargetPosition(ARM_BOTTOM_POSITION);
            armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void extendArmSlide()
    {
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlideMotor.setPower(0.7);
        armSlideMotor.setTargetPosition(ARM_EXTENDED_POSITION);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void retractArmSlide()
    {
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlideMotor.setPower(-0.7);
        armSlideMotor.setTargetPosition(ARM_RETRACTED_POSITION);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveArmSlide(double power)
    {
        armSlideMotor.setPower(power);
    }
}
