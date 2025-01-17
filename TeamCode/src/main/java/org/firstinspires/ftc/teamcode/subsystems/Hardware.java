package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    // Static hardware components
    private static DcMotor frontRightDrive, frontLeftDrive, backRightDrive, backLeftDrive, linearActuator, armSlideMotor, armRotationMotor;
    private static CRServo leftServo, rightServo;

    // Initialize drivetrain hardware
    // I think these need to be uppercase, but idk - Anthony
    public static void initDrivetrain(HardwareMap hardwareMap) {
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
    }

    // Initialize servo hardware
    public static void initServos(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "ls");
        rightServo = hardwareMap.get(CRServo.class, "rs");
    }

    // Initialize linear actuator
    public static void initLinearActuator(HardwareMap hardwareMap) {
        linearActuator = hardwareMap.get(DcMotor.class, "la1");
    }

    // Initialize arm hardware
    public static void initArm(HardwareMap hardwareMap) {
        armSlideMotor = hardwareMap.get(DcMotor.class, "asm");
        armRotationMotor = hardwareMap.get(DcMotor.class, "arm");
    }

    // Getters for hardware components
    public static DcMotor getFrontLeftDrive() {
        return frontLeftDrive;
    }

    public static DcMotor getBackLeftDrive() {
        return backLeftDrive;
    }

    public static DcMotor getFrontRightDrive() {
        return frontRightDrive;
    }

    public static DcMotor getBackRightDrive() {
        return backRightDrive;
    }

    public static DcMotor getLinearActuator() {
        return linearActuator;
    }

    public static DcMotor getArmSlideMotor() {
        return armSlideMotor;
    }

    public static DcMotor getArmRotationMotor() {
        return armRotationMotor;
    }

    public static CRServo getLeftServo() {
        return leftServo;
    }

    public static CRServo getRightServo() {
        return rightServo;
    }
}
