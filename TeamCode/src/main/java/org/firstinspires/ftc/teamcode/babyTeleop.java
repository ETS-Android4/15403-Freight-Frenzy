package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="teenageTeleop", group="Pushbot")
//@Disabled
public class babyTeleop extends LinearOpMode {

    public static final double UP_POSITION = .6;
    public static final double DOWN_POSITION = 1;
    public static final double OPEN_POSITION = .9;
    public static final double CLOSED_POSITION = .3;
    static final double SPIN = -1;

    babyHardwareMap robot = new babyHardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    private static int reverseDrive;

    public void runOpMode() {
        reverseDrive = 1;
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //driving
            if (gamepad1.right_trigger > 0.1){
                if(gamepad1.left_trigger > 0.1) { //Reverse
                    robot.leftDrive.setPower(gamepad1.right_stick_y * -1 / 2);
                    robot.rightDrive.setPower(gamepad1.left_stick_y * -1 / 2);
                }
                else { //Forward
                    robot.leftDrive.setPower(gamepad1.left_stick_y / 2);
                    robot.rightDrive.setPower(gamepad1.right_stick_y / 2);
                }
            }
            else {
                if(gamepad1.left_trigger > 0.1) { //Reverse
                    robot.leftDrive.setPower(gamepad1.right_stick_y * -1);
                    robot.rightDrive.setPower(gamepad1.left_stick_y * -1);
                }
                else { //Forward
                    robot.leftDrive.setPower(gamepad1.left_stick_y);
                    robot.rightDrive.setPower(gamepad1.right_stick_y);
                }
            }

            //Extend and retract the Delivery Arm to any position
            if (gamepad2.a) {
                robot.armmotor.setPower(-1);    //Extend
            } else if (gamepad2.b) {
                robot.armmotor.setPower(1);     //Retract
            } else {
                robot.armmotor.setPower(0);     //Stop Moving (Brake)
            }
            //Controls for the servo (Gate) that deposits a game element into a hub
            if (gamepad2.x){
                robot.armservo.setPosition(DOWN_POSITION);  //Drop gate
            }else if (gamepad2.y){
                robot.armservo.setPosition(UP_POSITION);    //Lift Gate

            }
            //Display encoder count on drive base motors for debugging
            telemetry.addData("left drive encoder", robot.leftDrive.getCurrentPosition());
            telemetry.addData("right drive encoder", robot.rightDrive.getCurrentPosition());


           /* else {
                robot.armservo.setPosition(10);

            }*/
            //===========
           // if(robot.intakemotor.getCurrentPosition()<=(-700)){
            //    robot.intakemotor.setPower(0);
           // }

            robot.intakemotor.setPower(-gamepad2.left_stick_y/4);


            if (gamepad2.right_bumper){
                robot.spinnermotor.setPower(1);
            }else if (gamepad2.left_bumper){
                robot.spinnermotor.setPower(-1);
            }else {
                robot.spinnermotor.setPower(0);
            }

            if (gamepad1.b){
                robot.duckmotor.setPower(SPIN);
            }
            else if (gamepad1.x){
                robot.duckmotor.setPower(-SPIN);
            }
            else {
                robot.duckmotor.setPower(0);
            }
            /*else if(gamepad1.dpad_left){
                robot.intakemotor.setPower(0.5);
            }
            else if (gamepad1.dpad_right){
                robot.intakemotor.setPower(-0.5);
            }
            else{
                robot.intakemotor.setPower(0);
            }*/
            //arm position telemetry
            //telemetry.addData("position",robot.intakeservo.getPosition());
            telemetry.update();
            telemetry.addData("intake pos", robot.intakemotor.getCurrentPosition());
            telemetry.update();

            if(gamepad1.dpad_up){
                robot.intakemotor.setTargetPosition(0);
                //robot.intakeservo.setPosition(OPEN_POSITION);
            }
        }


}}
