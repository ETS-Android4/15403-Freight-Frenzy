package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="babyTeleop", group="Pushbot")
//@Disabled
public class babyTeleop extends LinearOpMode {

    public static final double UP_POSITION = .6;
    public static final double DOWN_POSITION = 1;
    public static final double OPEN_POSITION = .9;
    public static final double CLOSED_POSITION = .3;
    static final double SPIN = 0.5;

    babyHardwareMap robot = new babyHardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //driving
            robot.leftDrive.setPower(gamepad1.left_stick_y);
            robot.rightDrive.setPower(gamepad1.right_stick_y);

            while (gamepad1.left_bumper){
                robot.leftDrive.setPower(gamepad1.left_stick_y/2);
                robot.rightDrive.setPower(gamepad1.right_stick_y/2);
            }


            //extends and retracts the linear slide arm.
            if (gamepad2.a) {
                robot.armmotor.setPower(-1);
            } else if (gamepad2.b) {
                robot.armmotor.setPower(1);
            } else {
                robot.armmotor.setPower(0);
            }
            while(robot.armmotor.getCurrentPosition()<=(0)){
                robot.armmotor.setPower(-1);}
            if(gamepad2.dpad_down){
                while(robot.intakemotor.getCurrentPosition()>=(0)){
                    robot.intakemotor.setPower(1);
                }
            }

            //controls the dropper servo with the x and y buttons
            if (gamepad2.x){
                robot.armservo.setPosition(DOWN_POSITION);
            }else if (gamepad2.y){
                robot.armservo.setPosition(UP_POSITION);

            }
            telemetry.addData("left drive encoder", robot.leftDrive.getCurrentPosition());
            telemetry.addData("right drive encoder", robot.rightDrive.getCurrentPosition());
           /* else {
                robot.armservo.setPosition(10);

            }*/
            //===========
            //coding for the intake motor
         robot.intakemotor.setPower(-gamepad2.left_stick_y/4);
            while(robot.intakemotor.getCurrentPosition()<=(-700)){
                robot.intakemotor.setPower(-1);
           }
            /*if (gamepad2.dpad_up){
                robot.intakemotor.setTargetPosition(0);
            }else if (gamepad2.dpad_down){
                robot.intakemotor.setTargetPosition(-700);
            }*/

            //open and close the intake servo
            if (gamepad2.right_bumper){
                robot.intakeservo.setPosition(OPEN_POSITION);
            }else if (gamepad2.left_bumper){
                robot.intakeservo.setPosition(CLOSED_POSITION);
            }


            //arm position telemetry
            telemetry.addData("position",robot.intakeservo.getPosition());
            telemetry.update();
            telemetry.addData("intake pos", robot.intakemotor.getCurrentPosition());
            telemetry.update();

            /*if(gamepad1.dpad_up){
                robot.intakemotor.setTargetPosition(0);
                robot.intakeservo.setPosition(OPEN_POSITION);
            }*/
        }


}}
