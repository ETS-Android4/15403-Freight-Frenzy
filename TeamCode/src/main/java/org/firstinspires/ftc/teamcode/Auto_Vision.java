package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto with Vision", group = "Pushbot")
public class Auto_Vision extends Auto_Util {
    babyHardwareMap robot = new babyHardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //Prepare vision
        initVision();
        useTwoRegions = true;
        setVisionRegionsA(1, 320, 1, 480);
        setVisionRegionB(321, 640, 1, 480);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Waiting for Start");    //
        telemetry.update();
        waitForStart();

        //=============
        //Vision code to determine path at beginning
        int path = 0;
        Bitmap bmp = getBarcodeBitmap();
        telemetry.addLine("AUtonomous program ahs te bitmap");
        telemetry.update();
        path = barcodeValue(bmp, RED);
        if(path == 1) {
            telemetry.addLine("Proceeding with Path 1");
        }
        else if(path == 2) {
            telemetry.addLine("Proceeding with Path 2");
        }
        else if(path == 3) {
            telemetry.addLine("Proceeding with Path 3");
        }
        else {
            telemetry.addLine("No path decided on.");
        }
        telemetry.update();
        sleep(10000);
    }
}
