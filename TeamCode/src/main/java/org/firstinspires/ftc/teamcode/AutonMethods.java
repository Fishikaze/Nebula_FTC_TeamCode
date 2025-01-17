package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.opencv.core.Mat;

public class AutonMethods {

    //use drivetrain instead of this?
    private static DcMotor frontRight, frontLeft, backRight, backLeft;

    private static Telemetry telemetry;

    public static void hardwareMap(Telemetry telemetry1) {
        telemetry = telemetry1;
        // While loop missing
        frontRight = Hardware.getFrontRightDrive();
        frontLeft = Hardware.getFrontLeftDrive();
        backLeft = Hardware.getBackLeftDrive();
        backRight = Hardware.getBackRightDrive();
    }

    //Does this even drive? The motor powers are never set (remove my changes if im wrong) - Anthony
    public static void drive(double theta, double power, double turn, boolean logs) {
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = power * cos/max + turn;
        double frontRightPower = power * sin/max - turn;
        double backLeftPower = power * sin/max + turn;
        double backRightPower = power * cos/max - turn;
        if((power + Math.abs(turn))>1) {
            frontLeftPower /= power + turn;
            frontRightPower /= power + turn;
            backLeftPower /= power + turn;
            backRightPower /= power + turn;
        }

        //Delete this if I'm wrong - Anthony
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        if (logs) {
            telemetry.addData("Front Left Motor Power", frontLeftPower);
            telemetry.addData("Front Right Motor Power", frontRightPower);
            telemetry.addData("Back Left Motor Power", backLeftPower);
            telemetry.addData("Back Right Motor Power", backRightPower);
            telemetry.update();
        }
    }
}
