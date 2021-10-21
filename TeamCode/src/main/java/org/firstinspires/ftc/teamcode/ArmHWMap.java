package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmHWMap extends HardwareMapUtil{
    HardwareMap hwmap = null;
    public DcMotor armmotor = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        armmotor = HardwareInitMotor("ArmMotor", true);
    }
}