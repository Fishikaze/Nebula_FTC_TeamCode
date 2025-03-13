package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auton with IMU and PID")


/*
    JASON:
    IMU AND PROPORTIONAL INTEGRAL DERIVATIVE CONTROLLLER (PID) IMPLEMENTED
    CAN BE REVERTED BACK TO BASIC ENCODER METHODS IF BUGGY / NON FUNCTIONING

    - TWEAK WITH PID CONSTANTS IN AUTON
    - CORRECTLY HARDWAREMAP THE IMU IN THE DS
    - TEST 1 FOOT METHOD TO ENSURE THAT IT WORKS
    - TEST GRIPPER METHOD; NOT 100% SURE IT IS IDEAL AND/OR FUNCTIONAL
    - CREATE ACTUAL AUTON

*/
public class AutonTest extends LinearOpMode {
    private DcMotor fl, fr, bl, br;
    private DcMotor armSlideMotor, armRotateMotor2, armRotateMotor1;
    private CRServo leftServo, rightServo;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    // PID controller constants
    private double kP = 0.03; // Proportional gain
    private double kI = 0.01; // Integral gain
    private double kD = 0.005; // Derivative gain

    // PID controller variables
    private double errorSum = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    public enum armPos { TOP, BOTTOM }
    public enum armRotatePos { TOP, BOTTOM }
    public enum intakeMode { INTAKE, OUTTAKE, STOP }

    public void doAuton() {
        // Example autonomous sequence
        driveStraight(0.7, 1000); // Drive forward
        sleep(1000);

        turnWithPID(90, 0.5); // Turn 90 degrees
        sleep(1000);

        driveStraight(0.5, 500); // Drive forward again
        sleep(1000);

        setIntakeMode(intakeMode.INTAKE);
        armSlidePosition(500, 0.3, armPos.TOP);
        setIntakeMode(intakeMode.OUTTAKE);
        armSlidePosition(500, 0.3, armPos.BOTTOM);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        armSlideMotor = hardwareMap.get(DcMotor.class, "asm");
        armRotateMotor1 = hardwareMap.get(DcMotor.class, "arm1");
        armRotateMotor2 = hardwareMap.get(DcMotor.class, "arm2");

        leftServo = hardwareMap.get(CRServo.class, "ls");
        rightServo = hardwareMap.get(CRServo.class, "rs");

        // Initialize IMU
        initIMU();

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotateMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior to brake for better control
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        resetAngle();
        doAuton();
    }

    // Initialize IMU
    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Make sure the IMU is calibrated
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("IMU Status", "Calibrated");
        telemetry.update();
    }

    // Reset angle tracking
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    // Get current angle from IMU
    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        // Handle angle wrapping (e.g., going from 179 to -179)
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    // PID controller for turning
    private double getPIDOutput(double error) {
        double currentTime = pidTimer.seconds();

        // If this is the first call to PID controller, just save the error and return 0
        if (currentTime == 0) {
            pidTimer.reset();
            lastError = error;
            return 0;
        }

        double deltaTime = currentTime;
        pidTimer.reset();

        // Calculate PID stuff
        double proportional = kP * error;

        errorSum += error * deltaTime;
        double integral = kI * errorSum;

        // ensure integral is limited to prevent builduip
        if (integral > 0.3) {
            integral = 0.3;
        } else if (integral < -0.3) {
            integral = -0.3;
        }

        double derivative = kD * (error - lastError) / deltaTime;
        lastError = error;

        // ensure output is between -1 and 1
        double output = proportional + integral + derivative;
        if (output > 1) {
            output = 1;
        } else if (output < -1) {
            output = -1;
        }

        return output;
    }

    // Turn using PID controller
    public void turnWithPID(double targetAngle, double maxPower) {
        // Reset PID controller variables
        errorSum = 0;
        lastError = 0;
        pidTimer.reset();

        double error;
        double output;

        // Continue turning until we're at the target angle with a small tolerance
        do {
            error = targetAngle - getAngle();
            output = getPIDOutput(error);

            // Scale output by maxPower
            output *= maxPower;

            // Set motor powers to turn
            fl.setPower(output);
            bl.setPower(output);
            fr.setPower(-output);
            br.setPower(-output);

            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", getAngle());
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.update();

        } while (opModeIsActive() && Math.abs(error) > 2); // 2 degree tolerance

        // Stop motors
        stopMotors();
    }

    // Drive straight using IMU to maintain heading
    public void driveStraight(double power, int distance) {
        // Reset encoders
        resetEncoders();

        // Set target positions
        fl.setTargetPosition(distance);
        fr.setTargetPosition(distance);
        bl.setTargetPosition(distance);
        br.setTargetPosition(distance);

        // Set run to position mode
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset angle tracking
        resetAngle();

        // Start motors
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        // While motors are moving, adjust to maintain heading
        while (opModeIsActive() &&
                (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {

            // Get current heading error
            double error = -getAngle(); // Negative because we want to correct in the opposite direction

            // Calculate correction factor (simple proportional control)
            double correction = error * 0.03; // Simple P control

            // Apply correction to motors
            fl.setPower(power - correction);
            fr.setPower(power + correction);
            bl.setPower(power - correction);
            br.setPower(power + correction);

            telemetry.addData("Target", distance);
            telemetry.addData("Position FL", fl.getCurrentPosition());
            telemetry.addData("Angle", getAngle());
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        // Stop motors
        stopMotors();
        resetMotors();
    }

    // Drive in any direction using mecanum wheels with IMU correction
    public void drive(double theta, double power, double turn, int distance) {
        resetEncoders();

        double sin = Math.sin(Math.toRadians(theta) - Math.PI / 4);
        double cos = Math.cos(Math.toRadians(theta) - Math.PI / 4);
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

        int flTarget = (int) (distance * (frontLeftPower / Math.max(maxPower, 1.0)));
        int frTarget = (int) (distance * (frontRightPower / Math.max(maxPower, 1.0)));
        int blTarget = (int) (distance * (backLeftPower / Math.max(maxPower, 1.0)));
        int brTarget = (int) (distance * (backRightPower / Math.max(maxPower, 1.0)));

        fl.setTargetPosition(flTarget);
        fr.setTargetPosition(frTarget);
        bl.setTargetPosition(blTarget);
        br.setTargetPosition(brTarget);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);

        // Reset angle tracking if we're not turning
        if (Math.abs(turn) < 0.1) {
            resetAngle();
        }

        // While motors are moving, adjust to maintain heading if we're not turning
        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {

            if (Math.abs(turn) < 0.1) {
                // Get current heading error
                double error = -getAngle(); // Negative because we want to correct in the opposite direction

                // Calculate correction factor (simple proportional control)
                double correction = error * 0.03; // Simple P control

                // Apply correction to motors, scaling by original power
                fl.setPower(frontLeftPower - correction * Math.abs(frontLeftPower));
                fr.setPower(frontRightPower + correction * Math.abs(frontRightPower));
                bl.setPower(backLeftPower - correction * Math.abs(backLeftPower));
                br.setPower(backRightPower + correction * Math.abs(backRightPower));
            }

            telemetry.addData("FL Target", flTarget);
            telemetry.addData("FL Position", fl.getCurrentPosition());
            telemetry.addData("Angle", getAngle());
            telemetry.update();
        }

        stopMotors();
        resetMotors();
    }

    // Stop all motors
    private void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    // Reset encoders
    private void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetMotors() {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                armRotateMotor1.setTargetPosition(armRotateMotor1.getCurrentPosition() + position);
                armRotateMotor2.setTargetPosition(armRotateMotor1.getCurrentPosition() + position);
                break;
            case BOTTOM:
                armRotateMotor1.setTargetPosition(armRotateMotor1.getCurrentPosition() - position);
                armRotateMotor2.setTargetPosition(armRotateMotor1.getCurrentPosition() + position);
                break;
        }
        armRotateMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor1.setPower(power);
        armRotateMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor2.setPower(power);
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
    public void moveOneFoot(double power) {
        final double TICKS_PER_INCH = 45.3; // Adjusted for 96mm Mecanum Wheels
        int targetTicks = (int) (12 * TICKS_PER_INCH); // One foot = 12 inches

        // Reset encoders
        resetEncoders();

        // Set target positions
        fl.setTargetPosition(targetTicks);
        fr.setTargetPosition(targetTicks);
        bl.setTargetPosition(targetTicks);
        br.setTargetPosition(targetTicks);

        // Set to RUN_TO_POSITION mode
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        // Wait until motion is complete
        while (opModeIsActive() && (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {
            telemetry.addData("Target", targetTicks);
            telemetry.addData("FL Position", fl.getCurrentPosition());
            telemetry.addData("FR Position", fr.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors
        stopMotors();

        // Reset motor modes
        resetMotors();
    }

}