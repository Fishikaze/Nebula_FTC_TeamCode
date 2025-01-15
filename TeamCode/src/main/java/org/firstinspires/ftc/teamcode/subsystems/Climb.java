package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climb {

    private DcMotor linearActuator;
    private Telemetry telemetry;

    public Climb(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        Hardware.initLinearActuator(hardwareMap);
        this.linearActuator = Hardware.getLinearActuator();

        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setLinearActuatorPosition(int position) {
        linearActuator.setTargetPosition(position);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setPower(1.0);

        while (linearActuator.isBusy() && telemetry != null) {
            telemetry.addData("Linear Actuator Position", linearActuator.getCurrentPosition());
            telemetry.addData("Target Position", position);
            telemetry.update();
        }

        linearActuator.setPower(0);
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void moveLinearActuator(double power) {
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setPower(power);
    }


    public void stopLinearActuator() {
        linearActuator.setPower(0);
    }
}
