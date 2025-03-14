package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "SigmaAuton")
public class AutonV2 extends LinearOpMode {
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftDrive = hardwareMap.get(DcMotor.class,"fl");
        frontRightDrive = hardwareMap.get(DcMotor.class,"fr");
        backLeftDrive = hardwareMap.get(DcMotor.class,"bl");
        backRightDrive = hardwareMap.get(DcMotor.class,"br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        frontLeftDrive.setPower(-0.4);
        frontRightDrive.setPower(-0.7);
        backLeftDrive.setPower(-0.4);
        backRightDrive.setPower(-0.7);
        sleep(3500);




    }
}


