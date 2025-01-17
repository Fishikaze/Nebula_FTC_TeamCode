package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutonTest extends LinearOpMode{
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private enum Direction {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT,
    }

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

        drive(500, 0.5, 0, Direction.FORWARD);
    }

    public void drive(int ticks, double power, double speed, Direction direction)
    {
        switch(direction) {
            case FORWARD:
                frontLeftDrive.setPower(power);
                frontRightDrive.setPower(power);
                backLeftDrive.setPower(power);
                backRightDrive.setPower(power);

                frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
                frontRightDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
                backLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
                backRightDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            case BACKWARD:
                frontLeftDrive.setPower(power);
                frontRightDrive.setPower(power);
                backLeftDrive.setPower(power);
                backRightDrive.setPower(power);

                frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);
                frontRightDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);
                backLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);
                backRightDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            case RIGHT:
                frontLeftDrive.setPower(power);
                frontRightDrive.setPower(power);
                backLeftDrive.setPower(power);
                backRightDrive.setPower(power);

                frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
                frontRightDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);
                backLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);
                backRightDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            case LEFT:
                frontLeftDrive.setPower(power);
                frontRightDrive.setPower(power);
                backLeftDrive.setPower(power);
                backRightDrive.setPower(power);

                frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);
                frontRightDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
                backLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
                backRightDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


    }

}