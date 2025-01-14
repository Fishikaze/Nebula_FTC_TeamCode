package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Hardware;

public class NebulaMainDrive
{
    private Telemetry telemetry;
    Hardware hardware = new Hardware();
    public Arm arm = new Arm(hardware.hardwareMapArm(new HardwareMap()), telemetry);
}
