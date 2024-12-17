package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Climb
{
    private DcMotor linearActuator1;
    private DcMotor linearActuator2;
    private Telemetry telemetry;

    //linearActuator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    public Climb(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        Hardware hardware = new Hardware();
        hardware.hardwareMapLinearActuators(hardwareMap);
        linearActuator1 = hardware.getLinearActuator1();
        linearActuator2 = hardware.getLinearActuator2();
    }
    public void setLinearActuatorPosition(int position, int linearActuator)
    {
        if (linearActuator == 1)
        {
            linearActuator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearActuator1.setPower(1);
            linearActuator1.setTargetPosition(position);
            linearActuator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(linearActuator == 2)
        {
            linearActuator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearActuator2.setPower(1);
            linearActuator2.setTargetPosition(position);
            linearActuator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

}
