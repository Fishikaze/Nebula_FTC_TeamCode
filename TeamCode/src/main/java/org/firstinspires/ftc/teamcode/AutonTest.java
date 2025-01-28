package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class AutonTest extends LinearOpMode{
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor armSlideMotor, armRotateMotor;
    private CRServo leftServo, rightServo;
    private final int ROTATE = 500;

    private enum Direction {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT,
    }
    private enum armPos{
        TOP,
        BOTTOM
    }
    private enum intakeMode
    {
        INTAKE,
        OUTTAKE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftDrive = hardwareMap.get(DcMotor.class,"fl");
        frontRightDrive = hardwareMap.get(DcMotor.class,"fr");
        backLeftDrive = hardwareMap.get(DcMotor.class,"bl");
        backRightDrive = hardwareMap.get(DcMotor.class,"br");

        armSlideMotor = hardwareMap.get(DcMotor.class, "asm");
        armRotateMotor = hardwareMap.get(DcMotor.class, "arm");

        leftServo = hardwareMap.get(CRServo.class, "ls");
        rightServo = hardwareMap.get(CRServo.class, "rs");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

//        drive(500, 0.5, Direction.FORWARD);
        setIntakeMode(intakeMode.INTAKE);
        armSlidePosition(500, 0.3, armPos.TOP);
        setIntakeMode(intakeMode.OUTTAKE);
        armSlidePosition(500, 0.3, armPos.BOTTOM);

    }
    public void setIntakeMode(intakeMode intakeMode)
    {
        switch (intakeMode)
        {
            case INTAKE:
                leftServo.setPower(1);
                rightServo.setPower(-1);
            case OUTTAKE:
                leftServo.setPower(-1);
                rightServo.setPower(1);

        }

    }
    public void armSlidePosition(int position, double power, armPos armPos)
    {
        switch (armPos)
        {
            case TOP:
                armSlideMotor.setPower(power);
                armSlideMotor.setTargetPosition(armSlideMotor.getCurrentPosition() + position);
                armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            case BOTTOM:
                armSlideMotor.setPower(-power);
                armSlideMotor.setTargetPosition(armSlideMotor.getCurrentPosition() - position);
                armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }
    public void drive(int ticks, double power, Direction direction)
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