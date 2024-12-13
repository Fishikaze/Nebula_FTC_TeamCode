package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain
{
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private Telemetry telemetry;
    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        Hardware hardware = new Hardware();
        hardware.hardwareMapMotors(hardwareMap);

        frontLeftDrive = hardware.getFrontLeftDrive();
        frontRightDrive = hardware.getFrontRightDrive();
        backLeftDrive = hardware.getBackLeftDrive();
        backRightDrive = hardware.getBackRightDrive();


    }
    public void teleopDrive(Gamepad gamepad1, double left_x, double left_y, double right_x, boolean logs)
    {
        double drive = gamepad1.left_stick_y * 0.5;
        double strafe = gamepad1.left_stick_x * 0.7;
        double spin = -gamepad1.right_stick_x * 0.4;


        frontLeftDrive.setPower(-drive + strafe - spin);
        frontRightDrive.setPower(-drive - strafe + spin);
        backLeftDrive.setPower(drive - strafe - spin);
        backRightDrive.setPower(drive + strafe + spin);

        if(logs)
        {
            telemetry.addData("Front Left Motor Power", frontLeftDrive.getPower());
            telemetry.addData("Front Right Motor Power", frontRightDrive.getPower());
            telemetry.addData("Back Left Motor Power", backLeftDrive.getPower());
            telemetry.addData("Back Right Motor Power", backRightDrive.getPower());
            telemetry.update();
        }

    }
}
