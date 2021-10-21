package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class babyHardwareMap extends HardwareMapUtil{
HardwareMap hwmap = null;
    public DcMotor examplemotor1 = null;
    public DcMotor examplemotor2 = null;
    public DcMotor armmotor = null;
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        armmotor =HardwareInitMotor("ArmMotor",true);
        examplemotor1=HardwareInitMotor("example1" , true);
        examplemotor2 =HardwareInitMotor("example2",true);
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
