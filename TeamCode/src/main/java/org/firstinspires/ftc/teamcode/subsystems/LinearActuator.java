package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class LinearActuator
{
    private DcMotor linearActuator;
    private Telemetry telemetry;
    //linearActuator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    public void extendLinearActuator(int extendAmount)
    {
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setPower(1);
        linearActuator.setTargetPosition(extendAmount);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void retractLinearActuator(int retractAmount)
    {

        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setPower(-1);
        linearActuator.setTargetPosition(retractAmount);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
