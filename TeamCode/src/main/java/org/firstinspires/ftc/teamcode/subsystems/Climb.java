package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Climb
{
    private DcMotor linearActuator;

    private Telemetry telemetry;

    //linearActuator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    public Climb(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        Hardware hardware = new Hardware();
        hardware.hardwareMapLinearActuator(hardwareMap);
        linearActuator = hardware.getLinearActuator();
    }
    public void setLinearActuatorPosition(int position, int linearActuator)
    {
        if (linearActuator == 1)
        {
            this.linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.linearActuator.setPower(1);
            this.linearActuator.setTargetPosition(position);
            this.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

}
