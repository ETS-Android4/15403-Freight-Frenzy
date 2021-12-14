package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class babyHardwareMap extends HardwareMapUtil{
HardwareMap hwmap = null;
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor armmotor = null;
    public DcMotor intakemotor = null;
    public Servo armservo = null;
    public Servo intakeservo = null;
    public DcMotor duckmotor = null;
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        armmotor =HardwareInitMotor("ArmMotor",true);
        leftDrive=HardwareInitMotor("leftDrive" , false);
        rightDrive =HardwareInitMotor("rightDrive",true);
        intakemotor = HardwareInitMotor("IntakeMotor",true);
        duckmotor = HardwareInitMotor("Duck", true);
        armservo = hwMap.get(Servo.class, "ArmServo");
        intakeservo = hwMap.get(Servo.class, "IntakeServo");
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //duckmotor = HardwareInitMotor("Duck", true);
    }
}
