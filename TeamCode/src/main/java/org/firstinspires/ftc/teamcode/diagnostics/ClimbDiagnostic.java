package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Hardware;

public class ClimbDiagnostic extends LinearOpMode {

    private DcMotor linearActuator;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.initLinearActuator(hardwareMap);

        linearActuator = Hardware.getLinearActuator();

        linearActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                setLinearActuatorPosition(10);
            }

            double POWER = 0.7;
            if (gamepad1.dpad_up) {
                linearActuator.setPower(POWER);
            } else if (gamepad1.dpad_down) {
                linearActuator.setPower(-POWER);
            } else {
                linearActuator.setPower(0);
            }

            telemetry.addLine("A → Set Linear Actuator Position");
            telemetry.addLine("Dpad → Control Linear Actuator power");
            telemetry.addData("Linear Actuator power: ", linearActuator.getPower());
            telemetry.update();
        }
    }


    public void setLinearActuatorPosition(int position) {
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setTargetPosition(position);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setPower(1);

        while (linearActuator.isBusy() && opModeIsActive()) {
            telemetry.addData("Target Position", position);
            telemetry.addData("Current Position", linearActuator.getCurrentPosition());
            telemetry.update();
        }

        linearActuator.setPower(0);
        linearActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
