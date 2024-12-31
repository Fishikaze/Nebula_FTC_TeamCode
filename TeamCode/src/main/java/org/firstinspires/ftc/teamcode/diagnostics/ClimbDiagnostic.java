package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;

public class ClimbDiagnostic extends LinearOpMode
{
    private DcMotor linearActuator1;
    private Telemetry telemetry;
    private final double POWER = 0.7;
    public void runOpMode() throws InterruptedException
    {
        Hardware hardware = new Hardware();
        hardware.hardwareMapDrivetrain(hardwareMap);

        linearActuator1 = hardware.getLinearActuator1();

        linearActuator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive())
        {
            if(gamepad1.a)
            {
                setLinearActuatorPosition(10,1);
            }
            linearActuator1.setPower(gamepad1.dpad_up ? POWER : 0);
            linearActuator1.setPower(gamepad1.dpad_down ? -POWER : 0);

            telemetry.addLine("A → Set Linear Actuator Position");
            telemetry.addLine("Dpad → Control Linear Actuator power");


            telemetry.addData("Linear Actuator power: ", linearActuator1.getPower());

            telemetry.update();
        }

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

    }

}


