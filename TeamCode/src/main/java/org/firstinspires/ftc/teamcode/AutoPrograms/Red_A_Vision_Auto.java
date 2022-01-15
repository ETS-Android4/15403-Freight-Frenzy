package org.firstinspires.ftc.teamcode.AutoPrograms;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Auto_Util;
import org.firstinspires.ftc.teamcode.babyHardwareMap;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Red A Vision Auto", group = "Pushbot")
public class Red_A_Vision_Auto extends Auto_Util {
    babyHardwareMap robot = new babyHardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.6;

    int maxAll = getColorInt(255, 255, 255, 255);
    int minAll = getColorInt(255, 0, 0, 0);
    int maxRed = getColorInt(255, 255, 150, 150);
    int minRed = getColorInt(255, 150, 0, 0);
    int maxBlue = getColorInt(255, 150, 255, 255);
    int minBlue = getColorInt(255, 100, 100, 100);
    int maxCap = getColorInt(255, 92, 255, 228);
    int minCap = getColorInt(255, 25, 181, 155);
    int maxYellow = getColorInt(255, 255, 255, 100);
    int minYellow = getColorInt(255, 180, 180, 0);

    public static final int RED = 1;
    public static final int BLUE = 2;
    public static final int CAP = 3;



    //Variables for Camera
    private static final String TAG = "Webcam Sample";
    private static final int secondsPermissionTimeout = 100;
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;
    private EvictingBlockingQueue<Bitmap> frameQueue;
    private Handler callbackHandler;
    public boolean useTwoRegions = false;
    int aMinX = 1;
    int aMaxX = 640;
    int aMinY = 1;
    int aMaxY = 480;
    //
    int bMinX = 1;
    int bMaxX = 640;
    int bMinY = 1;
    int bMaxY = 480;
    //
    int cMinX = 1;
    int cMaxX = 640;
    int cMinY = 1;
    int cMaxY = 480;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //initAuto();

        //Prepare vision
        useTwoRegions = true;
        setVisionRegionsA(1, 320, 1, 480);
        setVisionRegionB(321, 640, 1, 480);
        Bitmap bmp = null;

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addLine("Waiting for Start");    //
        telemetry.update();
        waitForStart();

        //======================== Start of Auto

        //Vision code to determine path at beginning of match
        int path = 0;
        bmp = getBarcodeBitmap();       //Get a bitmap (Pixel Array) from Camera
        telemetry.addLine("Auto program has bitmap");
        path = barcodeValue(bmp, RED);  //Decide on a path while on RED side of field from bitmap
        telemetry.addLine("Proceeding with Path: " + path);
        telemetry.update();
        sleep(200); //Sleep to quickly display chosen path before proceeding

        //==================== Start moving
        //Drive forward a small amount
        encoderDrive(DRIVE_SPEED,  2,  2, 5.0);
        sleep(500);
        //Pivot RIGHT 90 degrees
        encoderDrive(DRIVE_SPEED,  4.5,  -4.5, 5.0);
        sleep(500);
        //Drive reverse up to duck motor
        encoderDrive(0.6,  -6.5,  -6.5, 5.0);
        sleep(500);
        //Spin duck motor
        robot.duckmotor.setPower(-1);
        sleep(4000);
        //turn off the motor and drive forwards to line up with the goal
        robot.duckmotor.setPower(0);

        encoderDrive(DRIVE_SPEED,  16,  16, 5.0);
        sleep(500);
        //turn left towards the goal 90 degrees
        encoderDrive(DRIVE_SPEED,  -4.5,  4.5, 5.0);
        sleep(500);
        //if the element is on the right: deliver to top
        if(path == 3){
            //drive towards the goal a bit
            encoderDrive(DRIVE_SPEED,  3.5,  3.5, 5.0);
            sleep(500);
            //extend the arm
            robot.armmotor.setPower(-1);
            sleep(2500);
            //stop extending and lower the servo
            robot.armmotor.setPower(0);
            robot.armservo.setPosition(1);
            //drive backwards to shake out block
            sleep(1000);
            encoderDrive(DRIVE_SPEED,-1,-1,5);
            sleep(1000);
            //raise the servo after the block falls down
            robot.armservo.setPosition(0.3);
            sleep(500);
            //lower the arm
            robot.armmotor.setPower(1);
            sleep(2000);
        }
        //element in the middle
        else if(path == 2){
            //drive towards the goal a bit
            encoderDrive(DRIVE_SPEED,  3.5,  3.5, 5.0);
            sleep(500);
            //extend the arm
            robot.armmotor.setPower(-1);
            sleep(1300);
            //stop extending and lower the servo
            robot.armmotor.setPower(0);
            robot.armservo.setPosition(1);
            //drive backwards to shake out block
            sleep(1000);
            encoderDrive(0.7,-1,-   1,5);
            sleep(1000);
            //raise the servo after the block falls down
            robot.armservo.setPosition(0.3);
            sleep(500);
            //lower the arm
            robot.armmotor.setPower(1);
            sleep(1200);
        }
        //element on the left deliver tpo bottom
        else if(path == 1){
            //drive towards the goal a bit
            encoderDrive(DRIVE_SPEED,  3.5,  3.5, 5.0);
            sleep(500);
            //extend the arm
            robot.armmotor.setPower(-1);
            sleep(750);
            //stop extending and lower the servo
            robot.armmotor.setPower(0);
            robot.armservo.setPosition(1);
            //drive backwards to shake out block
            sleep(1000);
            encoderDrive(0.7,-1,-1,5);
            sleep(1000);
            //raise the servo after the block falls down
            robot.armservo.setPosition(0.3);
            sleep(500);
            //lower the arm
            robot.armmotor.setPower(1);
            sleep(700);
        }
        //turn right again
        robot.armmotor.setPower(0);
        encoderDrive(DRIVE_SPEED,  -5,  5, 5.0);
        sleep(500);
        //drive backwards into the warehouse
        encoderDrive(DRIVE_SPEED,  -30,  -30, 5.0);
        sleep(500);


    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    public void setVisionRegionsA(int minX, int maxX, int minY, int maxY) {
        aMinX = minX;
        aMaxX = maxX;
        aMinY = minY;
        aMaxY = maxY;
    }

    public void setVisionRegionB(int minX, int maxX, int minY, int maxY) {
        bMinX = minX;
        bMaxX = maxX;
        bMinY = minY;
        bMaxY = maxY;
    }

    public void setVisionRegionsC(int minX, int maxX, int minY, int maxY) {
        cMinX = minX;
        cMaxX = maxX;
        cMinY = minY;
        cMaxY = maxY;
    }

    public int barcodeValue(Bitmap frameMap, int targetColor) {
        //Decide on color
        //Divide main bitmap into 3 subsets
        //Bitmap A
        //telemetry.addLine("Attempting to divide bitmap...");
        telemetry.update();
        int aHeight = aMaxY - aMinY;
        int aWidth = aMaxX - aMinX;
        Bitmap bitmapA = Bitmap.createBitmap(frameMap, aMinX, aMinY, aWidth, aHeight);
        if(bitmapA != null) {
            telemetry.addLine("bitmapA created.");
        }
        else {
            telemetry.addLine("Failed to create bitmapA");
        }
        //Bitmap B
        int bHeight = bMaxY - bMinY;
        int bWidth = bMaxX - bMinX;
        Bitmap bitmapB = Bitmap.createBitmap(frameMap, bMinX, bMinY, bWidth, bHeight);
        if(bitmapB != null) {
            telemetry.addLine("bitmapB created.");
        }
        else {
            telemetry.addLine("Failed to create bitmapB");
        }
        //Bitmap C
        int cHeight = cMaxY - cMinY;
        int cWidth = cMaxX - cMinX;
        Bitmap bitmapC = Bitmap.createBitmap(frameMap, cMinX, cMinY, cWidth, cHeight);
        if(bitmapC != null) {
            telemetry.addLine("bitmapC created.");
        }
        else {
            telemetry.addLine("Failed to create bitmapC");
        }

        telemetry.addLine("Bitmap divided. Attempting to count pixels...");
        telemetry.update();
        //Get how many pixels fall within target color for each bitmap
        //int aPixels = pixelsColor(bitmapA, targetColorMin, targetColorMax);
        int aPixels = newPixelsColorCount(bitmapA, targetColor);
        telemetry.addLine("aPixels has been counted.");
        //==========
        //sleep(10000);
        //==========
        int bPixels = newPixelsColorCount(bitmapB, targetColor);
        telemetry.addLine("bPixels has been counted.");
        int cPixels = newPixelsColorCount(bitmapC, targetColor);
        telemetry.addLine("cPixels has been counted.");

        telemetry.addLine("Pixels counted. Attempting to compare counts");
        telemetry.update();
        telemetry.addLine("A_Pixels: " + aPixels);
        telemetry.addLine("B_Pixels: " + bPixels);
        if(useTwoRegions) {
            telemetry.addLine("Total Pixels (c): " + cPixels);
        }
        else {
            telemetry.addLine("C_Pixels: " + cPixels);
        }
        telemetry.update();
        //    sleep(10000);
        if(useTwoRegions == true) {
            if(aPixels < 500 && aPixels < bPixels) {
                return 2;
            }
            else if(bPixels < 500 && bPixels < aPixels) {
                return 3;
            }
            else {
                return 1;
            }
        }
        else {
            if(aPixels > bPixels && aPixels > cPixels) {
                return 1;
            }
            else if(bPixels > aPixels && bPixels > cPixels) {
                return 2;
            }
            else if(cPixels > bPixels && cPixels > aPixels) {
                return 3;
            }
        }
        return 0;
    }

    public int getColorInt(int alphaVal, int redVal, int greenVal, int blueVal) {
        int combColor = (alphaVal & 0xff) << 24 | (redVal & 0xff) << 16 | (greenVal & 0xff) << 8 | (blueVal & 0xff);
        return combColor;
    }

    public int newPixelsColorCount(Bitmap frameMap, int color) {
        int pixelCount = 0;
        if(color == 1) {//RED
            int minR = red(minRed);
            int minG = green(minRed);
            int minB = blue(minRed);
            int maxR = red(maxRed);
            int maxG = green(maxRed);
            int maxB = blue(maxRed);
            for(int i = 1; i < frameMap.getHeight() - 1; i += 2) {
                for(int j = 1; j < frameMap.getWidth() - 1; j += 2) {
                    int curPixel = frameMap.getPixel(j, i);
                    int pR = red(curPixel);
                    int pG = green(curPixel);
                    int pB = blue(curPixel);
                    if(pR >= minR && pR <= maxR) {
                        if(pG >= minG && pG <= maxG) {
                            if(pB >= minB && pB <= maxB) {
                                pixelCount++;
                            }
                        }
                    }
                }
            }
        }
        else if(color == 2) {//BLUE
            int minR = red(minBlue);
            int minG = green(minBlue);
            int minB = blue(minBlue);
            int maxR = red(maxBlue);
            int maxG = green(maxBlue);
            int maxB = blue(maxBlue);
            for(int i = 1; i < frameMap.getHeight() - 1; i += 2) {
                for(int j = 1; j < frameMap.getWidth() - 1; j += 2) {
                    int curPixel = frameMap.getPixel(j, i);
                    int pR = red(curPixel);
                    int pG = green(curPixel);
                    int pB = blue(curPixel);
                    if(pR >= minR && pR <= maxR) {
                        if(pG >= minG && pG <= maxG) {
                            if(pB >= minB && pB <= maxB) {
                                if(pB > pR + 20 && pB > pG + 20) {
                                    pixelCount++;
                                }
                            }
                        }
                    }
                }
            }
        }
        else if(color == 3) {//CAP

        }
        //telemetry.addLine("Color Values retrieved. Proceeding to count pixels...");
        //int pix = frameMap.getPixel(320, 240);
        //telemetry.addLine("Pixel RGB: " + red(pix) + " / " + blue(pix) + " / " + green(pix));
        telemetry.addLine("Pixels counted: " + pixelCount);
        telemetry.update();
        //sleep(10000);
        return pixelCount;
    }

    public Bitmap getBarcodeBitmap() {

        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        initializeFrameQueue(2);
        //AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        Bitmap bmp = null;

        try {
            //telemetry.addLine("Attempting to Open Camera...");
            openCamera();
            if (camera == null) return null;
            //telemetry.addLine("Camera Opened. Attempting to Start Camera...");
            startCamera();
            if (cameraCaptureSession == null) return null;
            telemetry.addLine("Camera Started. Attempting to pull bmp from poll...");
            telemetry.update();
            while(true) {
                bmp = frameQueue.poll();
                if (bmp != null) {
                    //onNewFrame(bmp);
                    telemetry.addLine("bitmap pulled from camera");
                    break;
                }
            }
            telemetry.update();
        } finally {
            closeCamera();
            telemetry.addLine("Camera Close.");
            telemetry.update();
        }
        return bmp;
    }

    private void onNewFrame(Bitmap frame) {
        //saveBitmap(frame);
        //frame.recycle(); // not strictly necessary, but helpful
    }



    //----------------------------------------------------------------------------------------------
    // Camera operations
    //----------------------------------------------------------------------------------------------

    private void initializeFrameQueue(int capacity) {
        /** The frame queue will automatically throw away bitmap frames if they are not processed
         * quickly by the OpMode. This avoids a buildup of frames in memory */
        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }

    private void openCamera() {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            error("camera not found or permission to use not granted: %s", cameraName);
        }
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return; // be idempotent

        /** YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
         * for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
         * image format supported by a camera */
        final int imageFormat = ImageFormat.YUY2;

        /** Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            error("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         * here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            /** Create a session in which requests to capture frames can be made */
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        /** The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                        /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                         * for the duration of the callback. So we copy here manually. */
                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                    @Override public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                        RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                    }
                                })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException |RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            error("exception starting camera");
            synchronizer.finish(null);
        }

        /** Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /** Retrieve the created session. This will be null on error. */
        cameraCaptureSession = synchronizer.getValue();
    }

    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    private void closeCamera() {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Utilities
    //----------------------------------------------------------------------------------------------

    private void error(String msg) {
        telemetry.log().add(msg);
        telemetry.update();
    }
    private void error(String format, Object...args) {
        telemetry.log().add(format, args);
        telemetry.update();
    }

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }
}