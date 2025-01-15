package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Hardware;

/**(
 * Jason Chang Mecanum Drive
 * Robotics team Nebula
 * V1.0
 *
 */
@TeleOp(name = "Mecanum First Test")
public class MainTeleOp extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private DcMotor armMotor;

    final private int BOTTOM = 0;

    final private int MIDDLE = 250;

    final private  int TOP = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        // While loop missing
        AutonMethods autonMethods = new AutonMethods();
        Hardware hardware = new Hardware();
        autonMethods.hardwareMap(hardwareMap);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive()) {

            double drive = gamepad1.left_stick_y * 0.5;
            double strafe = -gamepad1.left_stick_x * 0.7;
            double spin = gamepad1.right_stick_x * 0.4;

            frontLeft.setPower(drive + strafe - spin);
            frontRight.setPower(drive - strafe + spin);
            backLeft.setPower(drive - strafe - spin);
            backRight.setPower(drive + strafe + spin);

            if (gamepad1.dpad_up)
            {
                armMotor.setPower(0.5);

            }
            else if (gamepad1.dpad_down)
            {
                armMotor.setPower(-0.5);

            }
            else
            {
                armMotor.setPower(0);
            }
            if (gamepad1.a)
            {
                armEncoderMovement(0.5, TOP, true);
            }
            if (gamepad1.b)
            {
                armEncoderMovement(0.5, BOTTOM, true);
            }

        }
    }

    public void armEncoderMovement(double power, int targetPosition, boolean logs)
    {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(power);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor.isBusy())
        {
            if (logs)
            {
                telemetry.addData("Target Pos", armMotor.getTargetPosition());
                telemetry.addData("Current Pos", armMotor.getCurrentPosition());
                telemetry.update();
            }
        }
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
