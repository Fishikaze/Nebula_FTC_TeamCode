package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Hardware;

public class WheelsDiagnostic extends LinearOpMode {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private final double POWER = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.initDrivetrain(hardwareMap);

        frontLeftDrive = Hardware.getFrontLeftDrive();
        frontRightDrive = Hardware.getFrontRightDrive();
        backLeftDrive = Hardware.getBackLeftDrive();
        backRightDrive = Hardware.getBackRightDrive();

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            frontLeftDrive.setPower(gamepad1.x ? POWER : 0);
            frontRightDrive.setPower(gamepad1.y ? POWER : 0);
            backLeftDrive.setPower(gamepad1.a ? POWER : 0);
            backRightDrive.setPower(gamepad1.b ? POWER : 0);

            telemetry.addLine("X → Front Left");
            telemetry.addLine("Y → Front Right");
            telemetry.addLine("A → Back Left");
            telemetry.addLine("B → Back Right");

            telemetry.addData("Front Left Power", frontLeftDrive.getPower());
            telemetry.addData("Front Right Power", frontRightDrive.getPower());
            telemetry.addData("Back Left Power", backLeftDrive.getPower());
            telemetry.addData("Back Right Power", backRightDrive.getPower());
            telemetry.update();
        }
    }
}
