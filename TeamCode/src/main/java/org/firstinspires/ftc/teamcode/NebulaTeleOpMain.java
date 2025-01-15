package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class NebulaTeleOpMain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //AutonMethods.hardwareMap(telemetry);
        Drivetrain.register(hardwareMap, telemetry);
        Arm.register(hardwareMap, telemetry);
        Climb.register(hardwareMap, telemetry);
        Intake.register(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            double drive = gamepad1.left_stick_y * 0.5;
            double strafe = -gamepad1.left_stick_x * 0.7;
            double spin = gamepad1.right_stick_x * 0.4;

            Drivetrain.mecanumDrive(drive, strafe, spin, false);

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

            // NOTE : VALUES FOR POSITION ARE NOT TESTED. FIGURE OUT ASAP - JASON
            if (gamepad2.a) {
                Climb.setLinearActuatorPosition(1000);
            } else if (gamepad2.b) {
                Climb.setLinearActuatorPosition(0);
            }

            if(gamepad1.x) {
                Intake.setIntakeMode("intake");
            } else if(gamepad1.y) {
                Intake.setIntakeMode("outtake");
            }
        }
    }
}
