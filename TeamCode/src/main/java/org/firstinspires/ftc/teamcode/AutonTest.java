package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auton")
public class AutonTest extends LinearOpMode {
    private DcMotor fl, fr, bl, br;
    private DcMotor armSlideMotor, armRotateMotor;
    private CRServo leftServo, rightServo;

    public enum armPos { TOP, BOTTOM }
    public enum armRotatePos { TOP, BOTTOM }
    public enum intakeMode { INTAKE, OUTTAKE, STOP }

    public void doAuton() {
        drive(90, 0.7, 0, 1000);
        sleep(5000);

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
        armRotateMotor = hardwareMap.get(DcMotor.class, "arm");

        leftServo = hardwareMap.get(CRServo.class, "ls");
        rightServo = hardwareMap.get(CRServo.class, "rs");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        doAuton();
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
                armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() + position);
                break;
            case BOTTOM:
                armRotateMotor.setTargetPosition(armRotateMotor.getCurrentPosition() - position);
                break;
        }
        armRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotateMotor.setPower(power);
    }

    public void drive(double theta, double power, double turn, int distance) {
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = power * cos/max + turn;
        double frontRightPower = power * sin/max - turn;
        double backLeftPower = power * sin/max + turn;
        double backRightPower = power * cos/max - turn;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if((power + Math.abs(turn))>1) {
            frontLeftPower /= power + turn;
            frontRightPower /= power + turn;
            backLeftPower /= power + turn;
            backRightPower /= power + turn;
        }

        fl.setTargetPosition((int)(distance * (Math.abs(frontLeftPower / maxPower))) + fl.getCurrentPosition());
        fr.setTargetPosition((int)(distance * (Math.abs(frontRightPower / maxPower))) + fr.getCurrentPosition());
        bl.setTargetPosition((int)(distance * (Math.abs(backLeftPower / maxPower))) + bl.getCurrentPosition());
        br.setTargetPosition((int)(distance * (Math.abs(backRightPower / maxPower))) + br.getCurrentPosition());

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
        while (fl.getPower() != 0 || fr.getPower() != 0 || bl.getPower() != 0 || br.getPower() != 0)
        {

            int flp = fl.getCurrentPosition();
            int frp = fr.getCurrentPosition();
            int blp = bl.getCurrentPosition();
            int brp = br.getCurrentPosition();

            fl.setPower(flp >= fl.getTargetPosition() - 10 && flp <= fl.getTargetPosition() + 10 ? 0 : frontLeftPower);
            fr.setPower(frp >= fr.getTargetPosition() - 10 && frp <= fr.getTargetPosition() + 10 ? 0 : frontRightPower);
            bl.setPower(blp >= bl.getTargetPosition() - 10 && blp <= bl.getTargetPosition() + 10 ? 0 : backLeftPower);
            br.setPower(brp >= br.getTargetPosition() - 10 && brp <= br.getTargetPosition() + 10 ? 0 : backRightPower);

        }
    }

    public void resetMotors() {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
}
