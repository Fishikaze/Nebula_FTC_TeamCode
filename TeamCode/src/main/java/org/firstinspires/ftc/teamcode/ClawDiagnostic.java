//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//// LS - Reverse RS - Forward | Intake
//// LS - Forward RS - Reverse | Outtake
//
//@TeleOp(name = "ClawDiagnostic")
//
//public class ClawDiagnostic extends LinearOpMode
//{
//    private CRServo leftServo, rightServo;
//    private final double POWER = 0.7;
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        leftServo = hardwareMap.get(CRServo.class, "LS");
//        rightServo = hardwareMap.get(CRServo.class, "RS");
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            if(gamepad1.right_trigger > 0)
//            {
//                leftServo.setPower(POWER);
//            }
//            if(gamepad1.left_trigger > 0)
//                rightServo.setPower(POWER);
//            }
//
//            if (gamepad1.a)
//            {
//                leftServo.setPower(POWER);
//                rightServo.setPower(POWER);
//            }
//            if (gamepad1.b)
//            {
//               leftServo.setPower(0);
//               rightServo.setPower(0);
//            }
//            if (gamepad1.x)
//            {
//                setIntakeMode("IN");
//
//            }
//            if (gamepad1.y)
//            {
//                setIntakeMode("OUT");
//
//            }
//            telemetry.addData("Left Servo Power", leftServo.getPower());
//            telemetry.addData("Right Servo Power", rightServo.getPower());
//
//            telemetry.addData("Left Servo Direction", leftServo.getDirection());
//            telemetry.addData("Right Servo Direction", rightServo.getDirection());
//
//            telemetry.update();
//        }
//
//    }
//
//    public void setIntakeMode(String mode)
//    {
//        if (mode.equals("IN"))
//        {
//            leftServo.setDirection(CRServo.Direction.REVERSE);
//            rightServo.setDirection(CRServo.Direction.FORWARD);
//        }
//        else if (mode.equals("OUT"))
//        {
//            leftServo.setDirection(CRServo.Direction.FORWARD);
//            rightServo.setDirection(CRServo.Direction.REVERSE);
//        }
//        if (leftServo.getPower() != 0)
//        {
//            leftServo.setPower(0);
//            leftServo.setPower(POWER);
//        }
//        if (rightServo.getPower() != 0)
//        {
//            rightServo.setPower(0);
//            rightServo.setPower(POWER);
//        }
//    }
//}
