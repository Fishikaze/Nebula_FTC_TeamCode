package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "auton")
public class Auton extends LinearOpMode {
    private DcMotor fl, fr, bl, br;
    private DcMotor armSlideMotor, armRotateMotor2, armRotateMotor1;
    private CRServo clawServo, gripperServo;  // Use CRServo for continuous servo control

    public enum armPos { TOP, BOTTOM }
    public enum armRotatePos { TOP, BOTTOM }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();
        doAuton();
    }

    private void initializeHardware() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        armSlideMotor = hardwareMap.get(DcMotor.class, "asm");
        armRotateMotor1 = hardwareMap.get(DcMotor.class, "arm");
        armRotateMotor2 = hardwareMap.get(DcMotor.class, "arm2");

        clawServo = hardwareMap.get(CRServo.class, "claw");  // Initialize CRServo for claw
        gripperServo = hardwareMap.get(CRServo.class, "gripper");  // Initialize CRServo for gripper

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, fl, fr, bl, br, armSlideMotor, armRotateMotor1, armRotateMotor2);
    }

    public void doAuton() {
        // for (int i = 0; i < 5; i++) {  // Clipping cycle
        // telemetry.addData("Clipping Cycle", i + 1);
        // telemetry.update();

        // drive(90, 0.7, 0, 1000);
        // sleep(500);

        controlClaw(true, 0.5, 500);  // Open the claw with 50% power for 1 second
        controlGripper(true, 0.5, 1000);  // Turn gripper up with 50% power for 1 second
        // armSlidePosition(500, 0.3, armPos.TOP);
        sleep(2000);
        controlClaw(false, 0.5, 500);  // Close the claw with 50% power for 1 second
        controlGripper(false, 0.5, 1000);  // Turn gripper down with 50% power for 1 second
        // armSlidePosition(500, 0.3, armPos.BOTTOM);
        //}
    }

    // Method to control the claw with continuous servo (using power for opening/closing)
    public void controlClaw(boolean open, double power, long time) {
        if (open) {
            clawServo.setPower(power);  // Open the claw with power
        }

        if (!open) {
            clawServo.setPower(-power);  // Close the claw with negative power
        }
        sleep(time);  // Wait for the specified time
        clawServo.setPower(0);  // Stop the claw servo after the action
    }

    // Method to control the gripper with continuous servo (using power for up/down)
    public void controlGripper(boolean up, double power, long time) {
        if (up) {
            gripperServo.setPower(power);  // Turn the gripper up with power
        }
        if (!up) {
            gripperServo.setPower(-power);  // Turn the gripper down with negative power
        }
        sleep(time);  // Wait for the specified time
        gripperServo.setPower(0);  // Stop the gripper servo after the action
    }

    public void armSlidePosition(int position, double power, armPos pos) {
        int target = armSlideMotor.getCurrentPosition() + (pos == armPos.TOP ? position : -position);
        moveMotorToPosition(armSlideMotor, target, power);
    }

    public void armRotatePosition(int position, double power, armRotatePos pos) {
        int target1 = armRotateMotor1.getCurrentPosition() + (pos == armRotatePos.TOP ? position : -position);
        int target2 = armRotateMotor2.getCurrentPosition() + (pos == armRotatePos.TOP ? position : -position);
        moveMotorToPosition(armRotateMotor1, target1, power);
        moveMotorToPosition(armRotateMotor2, target2, power);
    }

    public void drive(double theta, double power, double turn, int distance) {
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = power * cos / max + turn;
        double frontRightPower = power * sin / max - turn;
        double backLeftPower = power * sin / max + turn;
        double backRightPower = power * cos / max - turn;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        setTargetPosition(fl, frontLeftPower, distance);
        setTargetPosition(fr, frontRightPower, distance);
        setTargetPosition(bl, backLeftPower, distance);
        setTargetPosition(br, backRightPower, distance);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION, fl, fr, bl, br);

        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        while (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) {
            idle();
        }

        resetMotors();
    }

    public void resetMotors() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, fl, fr, bl, br);
    }

    private void moveMotorToPosition(DcMotor motor, int position, double power) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        while (motor.isBusy()) {
            idle();
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setTargetPosition(DcMotor motor, double power, int distance) {
        motor.setTargetPosition((int) (distance * (power / Math.abs(power))) + motor.getCurrentPosition());
    }

    private void setMotorMode(DcMotor.RunMode mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    private void setMotorPower(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
}


