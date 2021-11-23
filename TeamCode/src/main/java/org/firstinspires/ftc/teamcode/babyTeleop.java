package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="babyTeleop", group="Pushbot")
//@Disabled
public class babyTeleop extends LinearOpMode {

    public static final double UP_POSITION = .64;
    public static final double DOWN_POSITION = 1;

    babyHardwareMap robot = new babyHardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        //robot.armservo.setPosition(90);

        waitForStart();

        while (opModeIsActive()) {
            robot.examplemotor1.setPower(gamepad1.right_stick_y);
            robot.examplemotor2.setPower(gamepad1.left_stick_y);

            if (gamepad1.a) {
                robot.armmotor.setPower(-1);
            } else if (gamepad1.b) {
                robot.armmotor.setPower(1);
            } else {
                robot.armmotor.setPower(0);
            }
            if (gamepad1.x){
                robot.armservo.setPosition(DOWN_POSITION);
            }else if (gamepad1.y){
                robot.armservo.setPosition(UP_POSITION);

            }
           /* else {
                robot.armservo.setPosition(10);

            }*/

            if (gamepad1.dpad_up) {
                robot.intakemotor.setPower(1);
            }
            else if (gamepad1.dpad_down) {
                robot.intakemotor.setPower(-1);
            }
            else{
                robot.intakemotor.setPower(0);
            }
            if (gamepad1.right_bumper){
                robot.intakeservo.setPosition(DOWN_POSITION);
            }else if (gamepad1.left_bumper){
                robot.intakeservo.setPosition(UP_POSITION);

            }
            telemetry.addData("position",robot.armservo.getPosition());
            telemetry.update();
        }
    }
}
