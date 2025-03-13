package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@TeleOp(name = "NEBULA TELEOP")
public class NebulaTeleOpMain extends LinearOpMode {
    // IMU
    private BNO055IMU imu;
    private double robotHeading = 0;

    // PID constajnts
    private static final double kP = 0.03; // Proportional gain
    private static final double kI = 0.01; // Integral gain
    private static final double kD = 0.005; // Derivative gain

    // PID variables
    private double errorSum = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    private boolean fieldCentric = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // Register subsystems
        Drivetrain.register(hardwareMap, telemetry);
        Arm.register(hardwareMap, telemetry);
        Climb.register(hardwareMap, telemetry);
        Gripper.register(hardwareMap);

        initIMU();

        telemetry.addData("Field-Centric Mode", fieldCentric ? "ENABLED" : "DISABLED");
        telemetry.addData("Press", "B to toggle Field-Centric");
        telemetry.update();

        waitForStart();

        pidTimer.reset();

        while (opModeIsActive()) {
            updateHeading();

            if (gamepad1.b && !gamepad1.start) {
                fieldCentric = !fieldCentric;
                telemetry.addData("Field-Centric Mode", fieldCentric ? "ENABLED" : "DISABLED");
                telemetry.update();
                sleep(250); // Debounce
            }

            if (gamepad1.y && !gamepad1.start) {
                resetHeading();
                telemetry.addData("Heading", "RESET");
                telemetry.update();
                sleep(250); // Debounce
            }

            double drive = gamepad1.left_stick_y * 0.5;
            double strafe = -gamepad1.left_stick_x * 0.7;
            double spin = gamepad1.right_stick_x * 0.4;

            if (fieldCentric) {
                // Rotate the drive vector by the robot's heading
                double heading = Math.toRadians(-robotHeading);
                double temp = drive * Math.cos(heading) - strafe * Math.sin(heading);
                strafe = drive * Math.sin(heading) + strafe * Math.cos(heading);
                drive = temp;
            }

            // Apply PID correction if driving forward/backward with minimal turning and strafing
            if (Math.abs(drive) > 0.1 && Math.abs(spin) < 0.1 && Math.abs(strafe) < 0.1) {
                // Get PID correction to keep robot driving straight
                double correction = getPIDCorrection();

                // Apply correction to spin component
                spin += correction;
            } else {
                // Reset PID variables when not driving straight
                resetPID();
            }

            // Drive the robot
            Drivetrain.mecanumDrive(drive, strafe, spin, false);

            // Telemetry for debugging
            telemetry.addData("Mode", fieldCentric ? "Field Centric" : "Robot Centric");
            telemetry.addData("Heading", robotHeading);
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Spin", spin);

            // Arm controls
            if (gamepad2.dpad_up) {
                Arm.setArmLevel("TOP");
            } else if (gamepad2.dpad_down) {
                Arm.setArmLevel("BOTTOM");
            }

            if (gamepad2.right_trigger > 0) {
                Arm.moveArmSlide(0.7);
            } else if (gamepad2.left_trigger > 0) {
                Arm.moveArmSlide(-0.7);
            } else {
                Arm.moveArmSlide(0);
            }

            // Climb controls
            // NOTE : VALUES FOR POSITION ARE NOT TESTED. FIGURE OUT ASAP - JASON
            if (gamepad2.a) {
                Climb.setLinearActuatorPosition(1000);
            } else if (gamepad2.b) {
                Climb.setLinearActuatorPosition(0);
            }

            // Gripper controls
            if (gamepad1.x) Gripper.toggleHorizontal();
            if (gamepad1.y && gamepad1.start) Gripper.toggleVertical(); // Changed to require start + Y to not conflict with heading reset
            if (gamepad1.right_bumper) Gripper.move(true);
            else if (gamepad1.left_bumper) Gripper.move(false);

            telemetry.update();
        }
    }

    /**
     * Initialize the IMU
     */
    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("IMU Status", "Calibrating...");
        telemetry.update();

        // Make sure the IMU is calibrated
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("IMU Status", "Calibrated");
        telemetry.update();
    }

    /**
     * Update the robot's heading from the IMU
     */
    private void updateHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotHeading = angles.firstAngle;
    }

    /**
     * Reset the robot's heading (make current direction the "forward" direction)
     */
    private void resetHeading() {
        // A simple approach is to reset the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        // Make sure the IMU is calibrated
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(10);
            idle();
        }

        robotHeading = 0;
    }

    /**
     * Reset PID controller variables
     */
    private void resetPID() {
        errorSum = 0;
        lastError = 0;
        pidTimer.reset();
    }

    /**
     * Get PID correction for keeping the robot driving straight
     * @return Correction value to apply to turning
     */
    private double getPIDCorrection() {

        // USES IMU AND PID TO FIX OFFSET OF ROBOT

        double error = -robotHeading;

        double currentTime = pidTimer.seconds();
        double deltaTime = Math.max(0.001, currentTime);
        pidTimer.reset();

        double proportional = kP * error;

        errorSum += error * deltaTime;
        if (errorSum > 10) errorSum = 10;
        if (errorSum < -10) errorSum = -10;
        double integral = kI * errorSum;

        double derivative = 0;
        if (deltaTime > 0) {
            derivative = kD * (error - lastError) / deltaTime;
        }
        lastError = error;

        // Calculate and limit output
        double output = proportional + integral + derivative;
        if (output > 0.3) output = 0.3;
        if (output < -0.3) output = -0.3;

        return output;
    }
}