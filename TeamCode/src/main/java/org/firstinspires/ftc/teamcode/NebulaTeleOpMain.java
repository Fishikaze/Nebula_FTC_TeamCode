package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;




@TeleOp(name = "NEBULA TELEOP")
public class NebulaTeleOpMain extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        Arm.register(hardwareMap, telemetry);
        Drivetrain.register(hardwareMap, telemetry);
        Climb.register(hardwareMap, telemetry);
        Gripper.register(hardwareMap);

        waitForStart();

        boolean armSlide = false, climb = false, armRotate = false;
        while (opModeIsActive()) {

            Drivetrain.teleopDrive(gamepad1);


            if (gamepad2.dpad_up) {
                armRotate = Arm.setArmLevel("TOP");
            } else if (gamepad2.dpad_down) {
                armRotate = Arm.setArmLevel("BOTTOM");
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
                climb = Climb.setLinearActuatorPosition(1000);
            } else if (gamepad2.b) {
                climb = Climb.setLinearActuatorPosition(0);
            }


            //GRIPPER NEW
            if(gamepad1.x) Gripper.toggleHorizontal();
            if (gamepad1.y) Gripper.toggleVertical();
            if (gamepad1.right_bumper) Gripper.move(true);
            else if (gamepad1.left_bumper) Gripper.move(false);


            // avoid while loops in teleop
            if (armSlide) {
                // intentionally left blank
            }
            if (armRotate) {
                Arm.setArmLevel(Arm.rotStorage);
            }
            if (climb) {
                Climb.setLinearActuatorPosition(Climb.posStorage);
            }

        }
    }
}


