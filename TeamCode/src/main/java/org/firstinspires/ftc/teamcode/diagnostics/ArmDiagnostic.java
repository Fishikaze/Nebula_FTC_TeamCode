package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
public class ArmDiagnostic extends LinearOpMode
{
    private DcMotor armSlideMotor, armRotateMotor;
    private Telemetry telemetry;
    private final double POWER = .7;
    private final int ARM_TOP_POSITION = 10;
    private final int ARM_BOTTOM_POSITION = 0;
    private final int ARM_EXTENDED_POSITION = 10;
    private final int ARM_RETRACTED_POSITION = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware();
        hardware.hardwareMapArm(hardwareMap);

        armSlideMotor = hardware.getArmSlideMotor();
        armRotateMotor = hardware.getArmRotationMotor();



        armSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while(opModeIsActive())
        {
            if (gamepad1.a)
            {
                setArmLevel("TOP");
            }
            else if (gamepad1.b)
            {
                setArmLevel("BOTTOM");

            }
            if (gamepad1.x)
            {
                extendArmSlide();
            }
            else if(gamepad1.y)
            {
                retractArmSlide();
            }
            armSlideMotor.setPower(gamepad1.dpad_up ? POWER : 0);
            armSlideMotor.setPower(gamepad1.dpad_down ? -POWER : 0);
            armRotateMotor.setPower(gamepad1.left_bumper ? POWER : 0);
            armRotateMotor.setPower(gamepad1.right_bumper ? -POWER : 0);
            telemetry.addLine("Dpad → Arm Rotate Motor");
            telemetry.addLine("Bumpers → Front Right");

            telemetry.addData("Front Left Power", armRotateMotor.getPower());
            telemetry.addData("Front Right Power", armSlideMotor.getPower());

            telemetry.update();
        }

    }
    public void setArmLevel(String pos)
    {
        int currentPosition = armRotateMotor.getCurrentPosition();
        int targetPosition = (pos.equals("TOP") ? ARM_TOP_POSITION : ARM_BOTTOM_POSITION);
        if (currentPosition < targetPosition)
        {
            armRotateMotor.setPower(1);
            armRotateMotor.setTargetPosition(targetPosition);
            armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (currentPosition > targetPosition)
        {
            armRotateMotor.setPower(-0.7);
            armRotateMotor.setTargetPosition(targetPosition);
            armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            armRotateMotor.setPower(0);
        }

    }
    public void extendArmSlide()
    {
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlideMotor.setPower(0.7);
        armSlideMotor.setTargetPosition(ARM_EXTENDED_POSITION);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void retractArmSlide()
    {
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlideMotor.setPower(-0.7);
        armSlideMotor.setTargetPosition(ARM_RETRACTED_POSITION);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
