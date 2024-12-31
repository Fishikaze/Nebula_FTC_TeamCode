package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;

public class ClimbDiagnostic extends LinearOpMode {
    private DcMotor linearActuator;
    private Telemetry telemetry;
    private final double POWER = 0.7;

    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware();
        hardware.hardwareMapLinearActuator(hardwareMap);

        linearActuator = hardware.getLinearActuator();

        linearActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                setLinearActuatorPosition(10);
            }

            //This can be condensed
            linearActuator.setPower(gamepad1.dpad_up ? POWER : 0);
            linearActuator.setPower(gamepad1.dpad_down ? -POWER : 0);

            telemetry.addLine("A → Set Linear Actuator Position");
            telemetry.addLine("Dpad → Control Linear Actuator power");


            telemetry.addData("Linear Actuator power: ", linearActuator.getPower());

            telemetry.update();
        }

    }

    public void setLinearActuatorPosition(int position) {

        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setPower(1);
        linearActuator.setTargetPosition(position);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

}


