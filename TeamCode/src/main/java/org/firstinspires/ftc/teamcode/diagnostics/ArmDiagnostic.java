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
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware();
        hardware.hardwareMapDrivetrain(hardwareMap);

        armSlideMotor = hardware.getArmSlideMotor();
        armRotateMotor = hardware.getArmRotationMotor();



        armSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while(opModeIsActive())
        {
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
}
