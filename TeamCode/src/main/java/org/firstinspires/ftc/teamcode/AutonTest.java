package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auton")
public class AutonTest extends LinearOpMode {
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotor armSlideMotor, armRotateMotor;
    private CRServo leftServo, rightServo;

    private enum Direction { FORWARD, BACKWARD, RIGHT, LEFT }
    private enum armPos { TOP, BOTTOM }
    private enum armRotatePos { TOP, BOTTOM }
    private enum intakeMode { INTAKE, OUTTAKE, STOP }

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

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

        setIntakeMode(intakeMode.INTAKE);
        armSlidePosition(500, 0.3, armPos.TOP);
        setIntakeMode(intakeMode.OUTTAKE);
        armSlidePosition(500, 0.3, armPos.BOTTOM);
    }

    public void setIntakeMode(intakeMode mode) {
        switch (mode) {
            case INTAKE:
                leftServo.setPower(1);
                rightServo.setPower(-1);
                break;
            case OUTTAKE:
                leftServo.setPower(-1);
                rightServo.setPower(1);
                break;
            case STOP:
                leftServo.setPower(0);
                rightServo.setPower(0);
                break;
        }
    }

    public void armSlidePosition(int position, double power, armPos pos) {
        switch (pos) {
            case TOP:
                armSlideMotor.setTargetPosition(armSlideMotor.getCurrentPosition() + position);
                break;
            case BOTTOM:
                armSlideMotor.setTargetPosition(armSlideMotor.getCurrentPosition() - position);
                break;
        }
        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setPower(power);
    }

    public void armRotatePosition(int position, double power, armRotatePos pos) {
        switch (pos) {
            case TOP:
                armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() + position);
                break;
            case BOTTOM:
                armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() - position);
                break;
        }
        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor.setPower(power);
    }

    public void drive(int ticks, double power, Direction direction) {
        switch (direction) {
            case FORWARD:
                frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
                frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + ticks);
                backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + ticks);
                backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + ticks);
                break;
            case BACKWARD:
                frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);
                frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() - ticks);
                backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - ticks);
                backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() - ticks);
                break;
            case RIGHT:
                frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
                frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() - ticks);
                backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - ticks);
                backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + ticks);
                break;
            case LEFT:
                frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);
                frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + ticks);
                backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + ticks);
                backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() - ticks);
                break;
        }
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void robotFN() {
        armRotatePosition(1300, 0.3, armRotatePos.TOP);
        armSlidePosition(1000, 0.3, armPos.TOP);
        setIntakeMode(intakeMode.OUTTAKE);
        sleep(1000);
        setIntakeMode(intakeMode.STOP);
        armRotatePosition(1300, 0.3, armRotatePos.BOTTOM);
        armSlidePosition(1000, 0.3, armPos.BOTTOM);
    }
}
