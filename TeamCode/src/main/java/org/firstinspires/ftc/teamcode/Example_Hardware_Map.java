package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Example_Hardware_Map extends HardwareMapUtil{
HardwareMap hwmap = null;
    public DcMotor examplemotor1 = null;
    public DcMotor examplemotor2 = null;
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        examplemotor1 =HardwareInitMotor("example1",true);
        examplemotor2 =HardwareInitMotor("example2",true);
    }
}
