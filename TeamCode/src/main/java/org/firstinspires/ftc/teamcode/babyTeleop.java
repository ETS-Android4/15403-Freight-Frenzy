package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="babyTeleop", group="Pushbot")
//@Disabled
public class babyTeleop extends LinearOpMode {
    babyHardwareMap robot = new babyHardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            robot.examplemotor1.setPower(gamepad1.right_stick_y);
            robot.examplemotor2.setPower(-gamepad1.left_stick_y);

            if (gamepad1.a == true) {
                robot.armmotor.setPower(1);
            } else if (gamepad1.b = true) {
                robot.armmotor.setPower(-1);
            } else {
                robot.armmotor.setPower(0);
            }
        }
    }
}
