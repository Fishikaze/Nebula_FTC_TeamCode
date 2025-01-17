package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.subsystems.Hardware;

@TeleOp(name = "ClawDiagnostic")
public class IntakeDiagnostic extends LinearOpMode {

    private CRServo leftServo, rightServo;
    private final double POWER = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.initServos(hardwareMap);

        leftServo = Hardware.getLeftServo();
        rightServo = Hardware.getRightServo();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                leftServo.setPower(POWER);
            } else if (gamepad1.left_trigger > 0) {
                rightServo.setPower(POWER);
            } else {
                leftServo.setPower(0);
                rightServo.setPower(0);
            }

            if (gamepad1.a) {
                leftServo.setPower(POWER);
                rightServo.setPower(POWER);
            } else if (gamepad1.b) {
                leftServo.setPower(0);
                rightServo.setPower(0);
            }

            if (gamepad1.x) {
                setIntakeMode("IN");
            } else if (gamepad1.y) {
                setIntakeMode("OUT");
            }

            // Telemetry for debugging
            telemetry.addData("Left Servo Power", leftServo.getPower());
            telemetry.addData("Right Servo Power", rightServo.getPower());
            telemetry.update();
        }
    }

    public void setIntakeMode(String mode) {
        if (mode.equals("IN")) {
            leftServo.setDirection(CRServo.Direction.REVERSE);
            rightServo.setDirection(CRServo.Direction.FORWARD);
        } else if (mode.equals("OUT")) {
            leftServo.setDirection(CRServo.Direction.FORWARD);
            rightServo.setDirection(CRServo.Direction.REVERSE);
        }

        leftServo.setPower(POWER);
        rightServo.setPower(POWER);
    }
}
