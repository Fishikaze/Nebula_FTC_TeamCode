package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climb {

    private static DcMotor linearActuator;
    private static Telemetry telemetry;

    public static void register(HardwareMap hardwareMap, Telemetry telemetry1) {
        telemetry = telemetry1;

        Hardware.initLinearActuator(hardwareMap);
        linearActuator = Hardware.getLinearActuator();

        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public static void setLinearActuatorPosition(int position) {
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


    public static void moveLinearActuator(double power) {
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setPower(power);
    }


    public static void stopLinearActuator() {
        linearActuator.setPower(0);
    }
}
