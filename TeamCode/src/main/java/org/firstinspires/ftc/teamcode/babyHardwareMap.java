package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class babyHardwareMap extends HardwareMapUtil{
HardwareMap hwmap = null;
    public DcMotor examplemotor1 = null;
    public DcMotor examplemotor2 = null;
    public DcMotor armmotor = null;
    public DcMotor intakemotor = null;
    public Servo armservo = null;
    public Servo intakeservo = null;
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        armmotor =HardwareInitMotor("ArmMotor",true);
        examplemotor1=HardwareInitMotor("example1" , true);
        examplemotor2 =HardwareInitMotor("example2",true);
        intakemotor = HardwareInitMotor("IntakeMotor",true);
        armservo = hwMap.get(Servo.class, "ArmServo");
        armservo = hwMap.get(Servo.class, "IntakeServo");
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
