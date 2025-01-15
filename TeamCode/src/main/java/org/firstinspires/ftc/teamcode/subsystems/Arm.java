package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private static DcMotor armSlideMotor;
    private static DcMotor armRotateMotor;
    private static Telemetry telemetry;

    private static final int ARM_TOP_POSITION = 10;
    private static final int ARM_BOTTOM_POSITION = 0;
    private static final int ARM_EXTENDED_POSITION = 10;
    private static final int ARM_RETRACTED_POSITION = 0;

    public static void register(HardwareMap hardwareMap, Telemetry telemetry1) {
        telemetry = telemetry1;

        Hardware.initArm(hardwareMap);
        armSlideMotor = Hardware.getArmSlideMotor();
        armRotateMotor = Hardware.getArmRotationMotor();

        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public static void setArmLevel(String pos) {
        int targetPosition = pos.equals("TOP") ? ARM_TOP_POSITION : ARM_BOTTOM_POSITION;

        armRotateMotor.setTargetPosition(targetPosition);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor.setPower(targetPosition > armRotateMotor.getCurrentPosition() ? 1.0 : -0.7);

        while (armRotateMotor.isBusy() && telemetry != null) {
            telemetry.addData("Arm Position", armRotateMotor.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
        }

        armRotateMotor.setPower(0);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public static void extendArmSlide() {
        moveArmSlideToPosition(ARM_EXTENDED_POSITION, 0.7);
    }


    public static void retractArmSlide() {
        moveArmSlideToPosition(ARM_RETRACTED_POSITION, -0.7);
    }


    public static void moveArmSlide(double power) {
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlideMotor.setPower(power);
    }


    public static void stopTargetingArm() {
        armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotateMotor.setPower(0);
    }


    private static void moveArmSlideToPosition(int position, double power) {
        armSlideMotor.setTargetPosition(position);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setPower(power);

        while (armSlideMotor.isBusy() && telemetry != null) {
            telemetry.addData("Arm Slide Position", armSlideMotor.getCurrentPosition());
            telemetry.addData("Target Position", position);
            telemetry.update();
        }

        armSlideMotor.setPower(0);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
