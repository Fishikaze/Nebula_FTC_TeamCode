package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TeleOp Drive")

public class TeleOpDrive extends LinearOpMode{

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor armMotor;
    private DcMotor armSlideMotor;
    private DcMotor armRotateMotor;


    final private int BOTTOM = 0;
    final private int MIDDLE = 250;
    final private int TOP = 500;

    private CRServo leftServo;
    private CRServo rightServo;

    private final double POWER = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftDrive = hardwareMap.get(DcMotor.class,"fl");
        frontRightDrive = hardwareMap.get(DcMotor.class,"fr");
        backLeftDrive = hardwareMap.get(DcMotor.class,"bl");
        backRightDrive = hardwareMap.get(DcMotor.class,"br");

        leftServo = hardwareMap.get(CRServo.class,"ls");
        rightServo = hardwareMap.get(CRServo.class,"rs");


        armSlideMotor = hardwareMap.get(DcMotor.class, "armSlideMotor");
        armRotateMotor = hardwareMap.get(DcMotor.class, "armRotateMotor");

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        armRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y * 0.5;
            double strafe = gamepad1.left_stick_x * 0.7;
            double spin = -gamepad1.right_stick_x * 0.4;

            frontLeftDrive.setPower(-drive + strafe - spin);
            frontRightDrive.setPower(-drive - strafe + spin);
            backLeftDrive.setPower(drive - strafe - spin);
            backRightDrive.setPower(drive + strafe + spin);

            telemetry.addData("Front Left Motor Power", frontLeftDrive.getPower());
            telemetry.addData("Front Right Motor Power", frontRightDrive.getPower());
            telemetry.addData("Back Left Motor Power", backLeftDrive.getPower());
            telemetry.addData("Back Right Motor Power", backRightDrive.getPower());
            telemetry.update();


            if (gamepad1.dpad_up)
            {
                armRotateMotor.setPower(0.5);
            }
            else if (gamepad1.dpad_down)
            {
                armRotateMotor.setPower(-0.5);
            }
            else
            {
                armRotateMotor.setPower(0);
            }

            if (gamepad1.y)
            {
                armSlideMotor.setPower(0.5);
            }
            else if (gamepad1.a)
            {
                armSlideMotor.setPower(-0.5);
            }
            else
            {
                armSlideMotor.setPower(0);
            }


            // if (gamepad1.a)
            // {
            //     armEncoderMovement(0.5, TOP, true);
            // }
            // if (gamepad1.b)
            // {
            //     armEncoderMovement(0.5, BOTTOM, true);
            // }


            if(gamepad1.right_trigger > 0)
            {
                leftServo.setPower(POWER);
            }
            if(gamepad1.left_trigger > 0)
            {
                rightServo.setPower(POWER);
            }

            if (gamepad1.a)
            {
                leftServo.setPower(POWER);
                rightServo.setPower(POWER);
            }
            if(gamepad1.b)
            {
                leftServo.setPower(0);
                rightServo.setPower(0);
            }
            if(gamepad1.x)
            {
                setIntakeMode("intake");
            }
            if(gamepad1.y)
            {
                setIntakeMode("outtake");
            }

//            telemetry.addData("Left Servo Power", leftServo.getPower());
//            telemetry.addData("Right Servo Power", rightServo.getPower());
//
//            telemetry.addData("Left Servo Direction",leftServo.getDirection());
//            telemetry.addData("Right Servo Direction",rightServo.getDirection());

            telemetry.update();

        }

    }


    public void armEncoderMovement(double power, int targetPosition, boolean logs) {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(power);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor.isBusy()) {
            if (logs) {
                telemetry.addData("Target Pos", armMotor.getTargetPosition());
                telemetry.addData("Current Pos", armMotor.getCurrentPosition());
                telemetry.update();
            }
        }
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void setIntakeMode(String mode)
    {
        if(mode.equals("intake"))
        {
            leftServo.setDirection(CRServo.Direction.REVERSE);
            rightServo.setDirection(CRServo.Direction.FORWARD);
        }
        else if(mode.equals("outtake"))
        {
            leftServo.setDirection(CRServo.Direction.FORWARD);
            rightServo.setDirection(CRServo.Direction.REVERSE);
        }

        if(leftServo.getPower() != 0)
        {
            leftServo.setPower(0);
            leftServo.setPower(POWER);
        }
        if(rightServo.getPower() != 0)
        {
            rightServo.setPower(0);
            rightServo.setPower(POWER);
        }
    }

}



