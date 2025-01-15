package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    private static DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    private static Telemetry telemetry;

    public static void register(HardwareMap hardwareMap, Telemetry telemetry1) {
        telemetry = telemetry1;

        Hardware.initDrivetrain(hardwareMap);

        frontLeftDrive = Hardware.getFrontLeftDrive();
        frontRightDrive = Hardware.getFrontRightDrive();
        backLeftDrive = Hardware.getBackLeftDrive();
        backRightDrive = Hardware.getBackRightDrive();

        configureMotors();
    }


    private static void configureMotors() {
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public static void teleopDrive(Gamepad gamepad1, boolean logs) {
        double drive = -gamepad1.left_stick_y * 0.5;
        double strafe = gamepad1.left_stick_x * 0.7;
        double spin = -gamepad1.right_stick_x * 0.4; // Rotation

        double frontLeftPower = drive + strafe - spin;
        double frontRightPower = drive - strafe + spin;
        double backLeftPower = drive - strafe - spin;
        double backRightPower = drive + strafe + spin;

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        if (logs) {
            telemetry.addData("Front Left Motor Power", frontLeftPower);
            telemetry.addData("Front Right Motor Power", frontRightPower);
            telemetry.addData("Back Left Motor Power", backLeftPower);
            telemetry.addData("Back Right Motor Power", backRightPower);
            telemetry.update();
        }
    }

    //from autonmethods
    public static void mecanumDrive(double theta, double power, double turn, boolean logs) {
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = power * cos/max + turn;
        double frontRightPower = power * sin/max - turn;
        double backLeftPower = power * sin/max + turn;
        double backRightPower = power * cos/max - turn;
        if((power + Math.abs(turn))>1) {
            frontLeftPower /= power + turn;
            frontRightPower /= power + turn;
            backLeftPower /= power + turn;
            backRightPower /= power + turn;
        }

        //Delete this if I'm wrong - Anthony
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        if (logs) {
            telemetry.addData("Front Left Motor Power", frontLeftPower);
            telemetry.addData("Front Right Motor Power", frontRightPower);
            telemetry.addData("Back Left Motor Power", backLeftPower);
            telemetry.addData("Back Right Motor Power", backRightPower);
            telemetry.update();
        }
    }
}
