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
            robot.examplemotor1.setPower(gamepad1.right_stick_y);
            robot.examplemotor2.setPower(gamepad1.left_stick_y);

            //extends and retracts the linear slide arm.
            if (gamepad1.a) {
                robot.armmotor.setPower(-1);
            } else if (gamepad1.b) {
                robot.armmotor.setPower(1);
            } else {
                robot.armmotor.setPower(0);
            }

            //controls the dropper servo with the x and y buttons
            if (gamepad1.x){
                robot.armservo.setPosition(DOWN_POSITION);
            }else if (gamepad1.y){
                robot.armservo.setPosition(UP_POSITION);

            }
           /* else {
                robot.armservo.setPosition(10);

            }*/

            if (gamepad1.dpad_up) {
                robot.intakemotor.setPower(.7);
            }
            else if (gamepad1.dpad_down) {
                robot.intakemotor.setPower(-.7);
            }
            else{
                robot.intakemotor.setPower(0);
            }
            if (gamepad1.right_bumper){
                robot.intakeservo.setPosition(OPEN_POSITION);
            }else if (gamepad1.left_bumper){
                robot.intakeservo.setPosition(CLOSED_POSITION);

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
            telemetry.addData("position",robot.intakeservo.getPosition());
            telemetry.update();
        }
    }
}
