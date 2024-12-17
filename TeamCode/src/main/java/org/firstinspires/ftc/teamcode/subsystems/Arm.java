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

        armRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setArmLevel(String pos)
    {
        int currentPosition = armRotateMotor.getCurrentPosition();
        int targetPosition = (pos.equals("TOP") ? ARM_TOP_POSITION : ARM_BOTTOM_POSITION);
        if (currentPosition < targetPosition)
        {
            armRotateMotor.setPower(1);
            armRotateMotor.setTargetPosition(targetPosition);
            armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (currentPosition > targetPosition)
        {
            armRotateMotor.setPower(-0.7);
            armRotateMotor.setTargetPosition(targetPosition);
            armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            armRotateMotor.setPower(0);
        }
    }
    public void extendArmSlide()
    {
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlideMotor.setPower(0.7);
        armSlideMotor.setTargetPosition(ARM_EXTENDED_POSITION);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void retractArmSlide()
    {
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlideMotor.setPower(-0.7);
        armSlideMotor.setTargetPosition(ARM_RETRACTED_POSITION);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveArmSlide(double power)
    {
        armSlideMotor.setPower(power);
    }
    public void stopTargetingArm()
    {
        armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotateMotor.setPower(0);
    }
}
