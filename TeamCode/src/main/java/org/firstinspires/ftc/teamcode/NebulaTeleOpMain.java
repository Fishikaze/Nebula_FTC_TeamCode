package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
public class NebulaTeleOpMain extends LinearOpMode
{
    private AutonMethods autonMethods;
    private Arm arm;
    private Climb climb;
    @Override
    public void runOpMode() throws InterruptedException
    {

        autonMethods = new AutonMethods();
        autonMethods.hardwareMap(hardwareMap);

        arm = new Arm(hardwareMap, telemetry);
        climb = new Climb(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive())
        {
            double drive = gamepad1.left_stick_y * 0.5;
            double strafe = -gamepad1.left_stick_x * 0.7;
            double spin = gamepad1.right_stick_x * 0.4;

            autonMethods.drive(drive,strafe,spin);
            if (gamepad2.dpad_up)
            {
                arm.setArmLevel("TOP");
            }
            else if (gamepad2.dpad_down)
            {
                arm.setArmLevel("BOTTOM");
            }


            if (gamepad2.right_trigger > 0)
            {
                arm.moveArmSlide(0.7);
            }
            else if (gamepad2.left_trigger > 0)
            {
                arm.moveArmSlide(-0.7);
            }
            else
            {
                arm.moveArmSlide(0);
            }

            // NOTE : VALUES FOR POSITION ARE NOT TESTED. FIGURE OUT ASAP - JASON
            if (gamepad2.a)
            {
                climb.setLinearActuatorPosition(1000);
            }
            else if (gamepad2.b)
            {
                climb.setLinearActuatorPosition(0);
            }
        }
    }
}
