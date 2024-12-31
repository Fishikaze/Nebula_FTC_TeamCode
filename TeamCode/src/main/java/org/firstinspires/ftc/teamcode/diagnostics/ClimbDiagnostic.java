package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;

public class ClimbDiagnostic extends LinearOpMode
{
    private DcMotor linearActuator1, linearActuator2;
    private Telemetry telemetry;
    private final double POWER = 0.7;
    public void runOpMode() throws InterruptedException
    {
        Hardware hardware = new Hardware();
        hardware.hardwareMapDrivetrain(hardwareMap);

        linearActuator1 = hardware.getLinearActuator1();
        linearActuator2 = hardware.getLinearActuator2();

        linearActuator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearActuator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive())
        {

        }
    }

}


