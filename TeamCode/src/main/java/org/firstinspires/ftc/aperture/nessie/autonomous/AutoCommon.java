package org.firstinspires.ftc.aperture.nessie.autonomous;

/**
 * Basic autonomous code shared by all autonomous programs.
 *
 * @author Aperture Science
 */

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Point;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.aperture.libraries.PIDController;
import org.firstinspires.ftc.aperture.libraries.VuforiaLocalizerImplSubclass;
import org.firstinspires.ftc.aperture.nessie.RobotHardware;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

public class AutoCommon extends LinearOpMode {

    protected RobotHardware robot;
    private ElapsedTime elapsedTime;
    protected ElapsedTime runTimer;

    @Override
    public void runOpMode() {

        robot = new RobotHardware(hardwareMap, false);

        telemetry.addData("WAIT", "robot is initializing.");
        telemetry.update();

        // set the servo positions
        robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_UP);
        robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_UP);
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INIT);
        robot.servoGrip.setPosition(robot.GRIP_INIT);
        robot.servoCap.setPosition(robot.CAP_IN_POSITION);
        robot.servoCapGripper.setPosition(robot.CAP_GRIPPER_CLOSED);

        telemetry.addData("WAIT", "robot is initializing.");
        if (robot.calibratedIMU) {
            telemetry.addData("IMU", "calibrated");
        } else {
            telemetry.addData("IMU", "NOT calibrated");
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Turn on the LEDs
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        for (int i = 0; i < 3; i++) {           // need multiple calls
            robot.ledStrip.setBrightness(31);   // 31 is full on
            robot.ledStrip.setColor(Color.parseColor("white"));
            sleep(50);
        }

        initVuforia();
        int which = findSkystoneRedBitmap();
        String msg = "skystone currently on the ";
        if (which == 6) {
            msg += "right";
        } else if (which == 5) {
            msg += "center";
        } else {
            msg += "left";
        }
        telemetry.addData("VISION", msg + " position");

        elapsedTime = new ElapsedTime();
        runTimer = new ElapsedTime();

        telemetry.addData("READY", "robot done initializing.");
        telemetry.update();

        waitForStart();

        runTimer.reset();
    }


    /**
     * Scissor Lift
     *
     * Move the scissor lift to a given height.
     */
    public final static int SCISSOR_SLOW_DOWN_DIST = 200;

    public void setScissorHeight(int targetHeight) {

        if (!opModeIsActive()) return;

        // where is the scissor currently?
        int currHeightL = robot.motorScissorL.getCurrentPosition();
        int currHeightR = robot.motorScissorR.getCurrentPosition();

        // are we going up or down?
        double dirL = Math.signum(targetHeight - currHeightL);
        double dirR = Math.signum(targetHeight - currHeightR);

        // are we there yet?
        boolean hasReachedHeightL = false;
        boolean hasReachedHeightR = false;

        while(opModeIsActive() && !(hasReachedHeightL && hasReachedHeightR)) {

            currHeightL = robot.motorScissorL.getCurrentPosition();
            currHeightR = robot.motorScissorR.getCurrentPosition();

            // control the left side of the scissor
            double scissorPowerL = 0;
            if ((targetHeight - currHeightL) * dirL <= 0) {
                hasReachedHeightL = true;
            } else {
                scissorPowerL = (dirL > 0) ? robot.SCISSOR_UP_SPEED : robot.SCISSOR_DOWN_SPEED;
                if(Math.abs(targetHeight - currHeightL) <= SCISSOR_SLOW_DOWN_DIST) {
                    scissorPowerL *= 0.5;
                }
            }

            if(dirL < 0 && !robot.touchScissorL.getState()){
                robot.motorScissorL.setPower(0);
                hasReachedHeightL = true;
            } else {
                robot.motorScissorL.setPower(scissorPowerL * dirL);
            }

            // control the right side of the scissor
            double scissorPowerR = 0;
            if ((targetHeight - currHeightR) * dirR <= 0) {
                hasReachedHeightR = true;
            } else {
                scissorPowerR = (dirR > 0) ? robot.SCISSOR_UP_SPEED : robot.SCISSOR_DOWN_SPEED;
                if(Math.abs(targetHeight - currHeightR) <= SCISSOR_SLOW_DOWN_DIST) {
                    scissorPowerR *= 0.5;
                }
            }

            if(dirR < 0 && !robot.touchScissorR.getState()){
                robot.motorScissorR.setPower(0);
                hasReachedHeightR = true;
            } else {
                robot.motorScissorR.setPower(scissorPowerR * dirR);
            }
        }

        // make sure the motors are off
        robot.motorScissorL.setPower(0);
        robot.motorScissorR.setPower(0);
    }

    /*
     * Drive a given distance (in inches) at a given speed.
     */
    public void drive(double distance, double speed) {

        if (!opModeIsActive()) return;

        speed = Math.abs(speed);
        double direction = Math.signum(distance);
        double distanceInTicks = robot.convertInchesToTicks(Math.abs(distance));

        robot.resetDriveEncoders();
        robot.startDrive(speed * direction);
        while (opModeIsActive() && Math.abs(robot.motorBackLeft.getCurrentPosition()) < distanceInTicks) {
            // do nothing
        }
        robot.stop();
    }

    /*************************************************************
     * IMU code for turns
     *************************************************************/
    private double initialHeading;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    private double getAngle() {
        Orientation angles = robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    public  double getHeading() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle - initialHeading;
    }

    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     *
     * https://stemrobotics.cs.pdx.edu/node/7268
     */
    public void rotate(int degrees, double power, PIDController pidRotate)
    {
        if (!opModeIsActive()) return;

        // restart imu angle tracking.
        resetAngle();

        System.out.println("ROTATE: start angle: " + getAngle());

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= 0)
            {
                robot.startMotorL(power);
                robot.startMotorR(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.startMotorL(-power);
                robot.startMotorR(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.startMotorL(-power);
                robot.startMotorR(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.stop();

        System.out.println("ROTATE: final angle: " + getAngle());

        // wait for rotation to stop.
        sleep(250);

        // reset angle tracking on new heading.
        resetAngle();
    }


    public void rotateAndSetScissorHeightUp(int degrees, double power, PIDController pidRotate, double targetHeight)
    {
        if (!opModeIsActive()) return;

        // restart imu angle tracking.
        resetAngle();

        System.out.println("ROTATE: start angle: " + getAngle());

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= 0)
            {
                robot.startMotorL(power);
                robot.startMotorR(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.startMotorL(-power);
                robot.startMotorR(power);
                setScissorHeightUp(targetHeight);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.startMotorL(-power);
                robot.startMotorR(power);
                setScissorHeightUp(targetHeight);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.stop();

        while (opModeIsActive() && !setScissorHeightUp(targetHeight)) {

        }

        System.out.println("ROTATE: final angle: " + getAngle());

        // wait for rotation to stop.
        sleep(250);

        // reset angle tracking on new heading.
        resetAngle();
    }

    /**
     * Drive using the IMU
     */
    private final static double DRIVE_GYRO_MULT_MOD = 0.05;
    private final static double DRIVE_GYRO_POW_MOD = 0.8;

    public void driveGyro(double distance, double speed, double targetHeading) {

        if (!opModeIsActive()) return;

        speed = Math.abs(speed);
        double direction = Math.signum(distance);
        double distanceInTicks = robot.convertInchesToTicks(Math.abs(distance));

        robot.resetDriveEncoders();
        while (opModeIsActive() && (Math.abs(robot.motorBackLeft.getCurrentPosition()) < distanceInTicks)) {

            // what is the error?
            double headingDiff = targetHeading - getHeading();
            if (headingDiff > 180) {
                headingDiff -= 360;
            } else if (headingDiff < -180) {
                headingDiff += 360;
            }

            // how big of a change in direction?
            double turn = Range.clip(-Math.signum(headingDiff) * DRIVE_GYRO_MULT_MOD *
                    Math.pow(Math.abs(headingDiff), DRIVE_GYRO_POW_MOD), -1.0, 1.0);

            // update the current drive
            robot.startMove(direction, 0, turn, speed);
        }

        System.out.println("DRIVE_GYRO: out of loop");
        System.out.println("DRIVE_GYRO: " + opModeIsActive());
        System.out.println("DRIVE_GYRO: " + (Math.abs(robot.motorBackLeft.getCurrentPosition()) < distanceInTicks));

        robot.stop();
    }
    public void driveGyro2(double distance, double speed, double targetHeading) {

        if (!opModeIsActive()) return;

        speed = Math.abs(speed);
        double direction = Math.signum(distance);
        double distanceInTicks = robot.convertInchesToTicks(Math.abs(distance));

        robot.resetDriveEncoders();
        while (opModeIsActive() && (Math.abs(robot.motorBackRight.getCurrentPosition()) < distanceInTicks)) {

            System.out.println("DRIVE_GYRO in loop");

            // what is the error?
            double headingDiff = targetHeading - getHeading();
            if (headingDiff > 180) {
                headingDiff -= 360;
            } else if (headingDiff < -180) {
                headingDiff += 360;
            }

            // how big of a change in direction?
            double turn = Range.clip(-Math.signum(headingDiff) * DRIVE_GYRO_MULT_MOD *
                    Math.pow(Math.abs(headingDiff), DRIVE_GYRO_POW_MOD), -1.0, 1.0);

            // update the current drive
            robot.startMove(direction, 0, turn, speed);
        }

        System.out.println("DRIVE_GYRO: out of loop");
        System.out.println("DRIVE_GYRO: " + opModeIsActive());
        System.out.println("DRIVE_GYRO: " + (Math.abs(robot.motorBackLeft.getCurrentPosition()) < distanceInTicks));

        robot.stop();
    }

    private final double SCISSOR_UP_SPEED = 0.8;
    private final double SCISSOR_DOWN_SPEED = 0.8;

    public boolean setScissorHeightUp(double targetHeight) {
        boolean hasReachedHeightL = false;
        boolean hasReachedHeightR = false;

        if (targetHeight > robot.motorScissorL.getCurrentPosition()) {
            robot.motorScissorL.setPower(SCISSOR_UP_SPEED);
        } else {
            robot.motorScissorL.setPower(0);
            hasReachedHeightL = true;
        }

        if (targetHeight > robot.motorScissorR.getCurrentPosition()) {
            robot.motorScissorR.setPower(SCISSOR_UP_SPEED);
        } else {
            robot.motorScissorR.setPower(0);
            hasReachedHeightR = true;
        }

        if (hasReachedHeightL && hasReachedHeightR) {
            return true;
        }

        return false;
    }

    public void driveGyroAndSetScissorHeightUp(double distance, double speed, double targetHeading, int targetHeight) {

        if (!opModeIsActive()) return;

        speed = Math.abs(speed);
        double direction = Math.signum(distance);
        double distanceInTicks = robot.convertInchesToTicks(Math.abs(distance));

        robot.resetDriveEncoders();
        while (opModeIsActive() && Math.abs(robot.motorBackLeft.getCurrentPosition()) < distanceInTicks) {

            // what is the error?
            double headingDiff = targetHeading - getHeading();
            if (headingDiff > 180) {
                headingDiff -= 360;
            } else if (headingDiff < -180) {
                headingDiff += 360;
            }

            // how big of a change in direction?
            double turn = Range.clip(-Math.signum(headingDiff) * DRIVE_GYRO_MULT_MOD *
                    Math.pow(Math.abs(headingDiff), DRIVE_GYRO_POW_MOD), -1.0, 1.0);

            // update the current drive
            robot.startMove(direction, 0, turn, speed);

            setScissorHeightUp(targetHeight);
        }
        robot.stop();

        while (opModeIsActive() && !setScissorHeightUp(targetHeight)) {

        }
    }

    public boolean setScissorHeightDown(double targetHeight) {

        // should these be remembered?
        boolean hasReachedHeightL = false;
        boolean hasReachedHeightR = false;

        if (targetHeight < robot.motorScissorL.getCurrentPosition() && robot.touchScissorL.getState()) {
            if(robot.motorScissorL.getCurrentPosition() < 700) {
                robot.motorScissorL.setPower(-SCISSOR_DOWN_SPEED/3.0);
            } else {
                robot.motorScissorL.setPower(-SCISSOR_DOWN_SPEED);
            }

        } else {
            robot.motorScissorL.setPower(0);
            hasReachedHeightL = true;
        }

        if (targetHeight < robot.motorScissorR.getCurrentPosition() && robot.touchScissorR.getState()) {
            if(robot.motorScissorR.getCurrentPosition() < 700) {
                robot.motorScissorR.setPower(-SCISSOR_DOWN_SPEED/3.0);
            } else {
                robot.motorScissorR.setPower(-SCISSOR_DOWN_SPEED);
            }

        } else {
            robot.motorScissorR.setPower(0);
            hasReachedHeightR = true;
        }

        if (hasReachedHeightL && hasReachedHeightR) {
            return true;
        }

        return false;
    }

    public void driveGyroAndSetScissorHeightDown(double distance, double speed, double targetHeading, int targetHeight) {

        if (!opModeIsActive()) return;

        speed = Math.abs(speed);
        double direction = Math.signum(distance);
        double distanceInTicks = robot.convertInchesToTicks(Math.abs(distance));

        robot.resetDriveEncoders();
        while (opModeIsActive() && Math.abs(robot.motorBackLeft.getCurrentPosition()) < distanceInTicks) {

            // what is the error?
            double headingDiff = targetHeading - getHeading();
            if (headingDiff > 180) {
                headingDiff -= 360;
            } else if (headingDiff < -180) {
                headingDiff += 360;
            }

            // how big of a change in direction?
            double turn = Range.clip(-Math.signum(headingDiff) * DRIVE_GYRO_MULT_MOD *
                    Math.pow(Math.abs(headingDiff), DRIVE_GYRO_POW_MOD), -1.0, 1.0);

            // update the current drive
            robot.startMove(direction, 0, turn, speed);

            setScissorHeightDown(targetHeight);
        }
        robot.stop();

        while (opModeIsActive() && !setScissorHeightDown(targetHeight)) {

        }
    }


    public void strafe(double power, double distance)
    {
        if (!opModeIsActive()) return;

        int ticks = (int)robot.convertInchesToTicks(Math.abs(distance));

        double direction = -Math.signum(distance);
        power = Math.abs(power);

        robot.resetDriveEncoders();
        robot.startStrafe(power * direction);
        while (opModeIsActive() && Math.abs(robot.motorBackRight.getCurrentPosition()) < ticks) {
        }
        robot.stop();
    }

    /*************************************************************
     * Vision code to find the Skystones
     *************************************************************/
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AT30c0j/////AAABmQwAYW6b3UgcszoTxj12AutsoJYh6fsfXXDWXjCbXLGDygqdPYi53"
                    +   "2meoJUTsVGKltZN/7fEqRv7Y1DZympsptLtTPyG9s0Xizp6YKafebGW2NALBbxNzQ2jSC"
                    +   "tohvdNCnTMY7ILT9xhcE3u1vLStOBdIy10G9PL0ISDkwv2MzteegEwzIaM4acZs/SF9LR"
                    +   "h4JP8jpPNyfZDUZCTSGp1D9UpagYfaProK/yenpSCuhypkhJ/6s01KFg2oVcfw9ZaQuR9"
                    +   "IfLFrdxXvLrgnrI/QZyhzA/OqQj24BHVT37BQvo8IyNKqZthTGpSziejt+ySvuW5n3AyH"
                    +   "o4wqeCnwYi6mMKrjYGL7sC8NVoV+qmU0CKC";

    private VuforiaLocalizerImplSubclass vuforia;
    private TFObjectDetector tfod;


    protected void turnOffVision() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webCam");

        vuforia = new VuforiaLocalizerImplSubclass(parameters);
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. For OpenRC, these are loaded from
        // the internal storage to reduce APK size
        /*
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromFile("/sdcard/FIRST/Skystone");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");
         */
    }

    private void waitForBitmap() {
        vuforia.copyNextToBitmap = true;
        //while (opModeIsActive() && vuforia.copyNextToBitmap) {
        while (vuforia.copyNextToBitmap) {
            // do nothing
        }
        System.out.println("VUFORIA: bitmap is ready");
    }

    private void saveBitmapToFile(Point leftTop, Point leftBottom, Point rightTop, Point rightBottom) {
        // draw the left box
        drawPointBox(vuforia.bitmap, leftTop.x, leftTop.y);
        drawRect(vuforia.bitmap, leftTop.x, leftTop.y, leftBottom.x, leftBottom.y);

        // draw the right box
        drawPointBox(vuforia.bitmap, rightTop.x, rightTop.y);
        drawRect(vuforia.bitmap, rightTop.x, rightTop.y, rightBottom.x, rightBottom.y);

        // store the image to a file
        storeImage(vuforia.bitmap);
    }

    private void storeImage(Bitmap image) {
        File pictureFile = getOutputMediaFile();
        if (pictureFile == null) {
            return;
        }
        try {
            FileOutputStream fos = new FileOutputStream(pictureFile);
            image.compress(Bitmap.CompressFormat.PNG, 100, fos);
            fos.close();
            System.out.println("VUFORIA: Image saved to " + pictureFile);
        } catch (FileNotFoundException e) {
            System.out.println("VUFORIA: File not found: " + e.getMessage());
        } catch (IOException e) {
            System.out.println("VUFORIA: Error accessing file: " + e.getMessage());
        }
    }

    /** Create a File for saving an image or video */
    private File getOutputMediaFile(){
        // To be safe, you should check that the SDCard is mounted
        // using Environment.getExternalStorageState() before doing this.
        File mediaStorageDir = new File(Environment.getExternalStorageDirectory()
                + "/Android/data/"
                + "org.firstinspires.ftc.org.firstinspires.ftc.aperture"
                + "/Files");

        // This location works best if you want the created images to be shared
        // between applications and persist after your app has been uninstalled.

        // Create the storage directory if it does not exist
        if (! mediaStorageDir.exists()){
            if (! mediaStorageDir.mkdirs()){
                System.out.println("VUFORIA: unable to save the file");
                return null;
            }
        }
        // Create a media file name
        String timeStamp = new SimpleDateFormat("ddMMyyyy_HHmm").format(new Date());
        File mediaFile;
        String mImageName="MI_"+ timeStamp +".png";
        mediaFile = new File(mediaStorageDir.getPath() + File.separator + mImageName);
        return mediaFile;
    }

    private void drawRect(Bitmap bm, float upperX, float upperY, float lowerX, float lowerY) {

        //System.out.println("VUFORIA: drawRect ==> X ==> " + upperX + " to " + lowerX);
        //System.out.println("VUFORIA: drawRect ==> Y ==> " + upperY + " to " + lowerY);

        // draw horizontally
        int min = Math.min((int)lowerX, (int)upperX);
        int max = Math.max((int)lowerX, (int)upperX);
        for (int x = min; x < max; x++) {
            bm.setPixel(x, (int)upperY, 0);
            bm.setPixel(x, (int)lowerY, 0);
        }

        // draw vertically
        min = Math.min((int)lowerY, (int)upperY);
        max = Math.max((int)lowerY, (int)upperY);
        for (int y = min; y < max; y++) {
            bm.setPixel((int)upperX, y, 0);
            bm.setPixel((int)lowerX, y, 0);
        }
    }

    private void drawPointBox(Bitmap bm, float xLoc, float yLoc) {
        for (int x = (int)xLoc - 3; x <= (int)xLoc + 3; x++) {
            for (int y = (int)yLoc - 3; y <= (int)yLoc + 3; y++) {
                bm.setPixel(x, y, 0xFF << 8);
            }
        }
    }

    private final Point RED_LEFT_TOP   = new Point(30, 270);
    private final Point RED_CENTER_TOP = new Point(240, 270);
    private final Point RED_RIGHT_TOP  = new Point(450, 270);

    public int findSkystoneRedBitmap() {
        // get a bitmap from the camera
        waitForBitmap();

        // left box
        int leftSum = 0;
        for (int x = 0; x < 60; x++) {
            for (int y = 0; y < 60; y++) {
                leftSum += (int) ((vuforia.bitmap.getPixel(x + RED_LEFT_TOP.x, y + RED_LEFT_TOP.y) >> 16) & 0xFF);
            }
        }
        System.out.println("SKYSTONE: left leftSum = " + leftSum);

        // center box
        int centerSum = 0;
        for (int x = 0; x < 60; x++) {
            for (int y = 0; y < 60; y++) {

                centerSum += (int) ((vuforia.bitmap.getPixel(x + RED_CENTER_TOP.x, y + RED_CENTER_TOP.y) >> 16) & 0xFF);
                //System.out.printf("SKYSTONE: %x\n", vuforia.bitmap.getPixel(x + RED_CENTER_TOP.x, y + RED_CENTER_TOP.y));
                //System.out.printf("SKYSTONE: %x\n", (vuforia.bitmap.getPixel(x + RED_CENTER_TOP.x, y + RED_CENTER_TOP.y) >> 16) & 0xFF);
            }
        }
        System.out.println("SKYSTONE: center leftSum = " + centerSum);

        // right box
        int rightSum = 0;
        for (int x = 0; x < 60; x++) {
            for (int y = 0; y < 60; y++) {
                rightSum += (int) ((vuforia.bitmap.getPixel(x + RED_RIGHT_TOP.x, y + RED_RIGHT_TOP.y) >> 16) & 0xFF);
            }
        }
        System.out.println("SKYSTONE: right leftSum = " + rightSum);


        //saveBitmapToFile(RED_LEFT_TOP, RED_LEFT_BOTTOM, RED_RIGHT_TOP, RED_RIGHT_BOTTOM);
        if (leftSum < centerSum && leftSum < rightSum) {
            return 4;
        } else if (centerSum < rightSum) {
            return 5;
        } else {
            return 6;
        }
    }

    private final Point BLUE_LEFT_TOP   = new Point(3, 3);
    private final Point BLUE_LEFT_BOTTOM  = new Point(100, 100);
    private final Point BLUE_RIGHT_TOP  = new Point(100, 100);
    private final Point BLUE_RIGHT_BOTTOM = new Point(200, 200);

    public int findSkystoneBlueBitmap() {
        // get a bitmap from the camera
        waitForBitmap();

        // left box
        int leftSum = 0;
        for (int x = 0; x < 60; x++) {
            for (int y = 0; y < 60; y++) {
                leftSum += (int) ((vuforia.bitmap.getPixel(x + RED_LEFT_TOP.x, y + RED_LEFT_TOP.y) >> 16) & 0xFF);
            }
        }
        System.out.println("SKYSTONE: left leftSum = " + leftSum);

        // center box
        int centerSum = 0;
        for (int x = 0; x < 60; x++) {
            for (int y = 0; y < 60; y++) {

                centerSum += (int) ((vuforia.bitmap.getPixel(x + RED_CENTER_TOP.x, y + RED_CENTER_TOP.y) >> 16) & 0xFF);
                //System.out.printf("SKYSTONE: %x\n", vuforia.bitmap.getPixel(x + RED_CENTER_TOP.x, y + RED_CENTER_TOP.y));
                //System.out.printf("SKYSTONE: %x\n", (vuforia.bitmap.getPixel(x + RED_CENTER_TOP.x, y + RED_CENTER_TOP.y) >> 16) & 0xFF);
            }
        }
        System.out.println("SKYSTONE: center leftSum = " + centerSum);

        // right box
        int rightSum = 0;
        for (int x = 0; x < 60; x++) {
            for (int y = 0; y < 60; y++) {
                rightSum += (int) ((vuforia.bitmap.getPixel(x + RED_RIGHT_TOP.x, y + RED_RIGHT_TOP.y) >> 16) & 0xFF);
            }
        }
        System.out.println("SKYSTONE: right leftSum = " + rightSum);


        //saveBitmapToFile(RED_LEFT_TOP, RED_LEFT_BOTTOM, RED_RIGHT_TOP, RED_RIGHT_BOTTOM);
        if (leftSum < centerSum && leftSum < rightSum) {
            return 6;
        } else if (centerSum < rightSum) {
            return 5;
        } else {
            return 4;
        }
    //saveBitmapToFile(BLUE_LEFT_TOP, BLUE_LEFT_BOTTOM, BLUE_RIGHT_TOP, BLUE_RIGHT_BOTTOM);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private final int MAX_POSITION_COUNT = 3;
    private final int TF_TIME_OUT = 1000;  // 2 seconds
    private final int SCREEN_RIGHT_OF_CENTER_BLUE = 325;
    private final int SCREEN_RIGHT_OF_CENTER_RED = 290;
    private final int SCREEN_CENTER = 0;

    public int findSkystoneRedTF() {

        if (tfod == null) {
            return -1;
        }

        int position6Count = 0;
        int position5Count = 0;
        int position4Count = 0;

        elapsedTime.reset();

        while (position6Count + position5Count + position4Count < MAX_POSITION_COUNT
                && elapsedTime.milliseconds() < TF_TIME_OUT
                && opModeIsActive()) {

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {

                System.out.println("FIND_SKYSTONE: # objects detected: " + updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {

                    if (!opModeIsActive()) break;

                    // look only for skystones
                    if (recognition.getLabel().equals("Skystone")) {
                        double center = (recognition.getRight() + recognition.getLeft()) / 2.0;
                        System.out.println("FIND_SKYSTONE: center: " + center);
                        System.out.println("FIND_SKYSTONE: confidence " + recognition.getConfidence());

                        if (center >= SCREEN_RIGHT_OF_CENTER_RED) {
                            position6Count++;
                        } else if (center >= SCREEN_CENTER) {
                            position5Count++;
                        } else {
                            position4Count++;
                        }

                    }
                    telemetry.update();
                }
            }

            sleep(50);
        }

        tfod.shutdown();

        System.out.println("FIND_SKYSTONE: position6Count = " + position6Count);
        System.out.println("FIND_SKYSTONE: position5Count = " + position5Count);
        System.out.println("FIND_SKYSTONE: position4Count = " + position4Count);
        System.out.println("FIND_SKYSTONE: elapsedTime = " + elapsedTime.milliseconds());

        if (position6Count > position5Count && position6Count > position4Count) {
            return 6;
        } else if (position5Count > position4Count) {
            return 5;
        }
        return 4;
    }

    public int findSkystoneBlueTF() {

        if (tfod == null) {
            return -1;
        }

        int position6Count = 0;
        int position5Count = 0;
        int position4Count = 0;

        elapsedTime.reset();

        while (position6Count + position5Count + position4Count < MAX_POSITION_COUNT
                && elapsedTime.milliseconds() < TF_TIME_OUT
                && opModeIsActive()) {

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {

                System.out.println("FIND_SKYSTONE: # objects detected: " + updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {

                    if (!opModeIsActive()) break;

                    // look only for skystones
                    if (recognition.getLabel().equals("Skystone")) {
                        double center = (recognition.getRight() + recognition.getLeft()) / 2.0;
                        System.out.println("FIND_SKYSTONE: center: " + center);
                        System.out.println("FIND_SKYSTONE: confidence " + recognition.getConfidence());


                        if (center >= SCREEN_RIGHT_OF_CENTER_BLUE) {
                            position5Count++;
                        } else if (center >= SCREEN_CENTER) {
                            position6Count++;
                        } else {
                            position4Count++;
                        }

                    }
                    telemetry.update();
                }
            }

            sleep(50);
        }

        tfod.shutdown();

        System.out.println("FIND_SKYSTONE: position6Count = " + position6Count);
        System.out.println("FIND_SKYSTONE: position5Count = " + position5Count);
        System.out.println("FIND_SKYSTONE: position4Count = " + position4Count);
        System.out.println("FIND_SKYSTONE: elapsedTime = " + elapsedTime.milliseconds());

        //if (position5Count == 1 && position4Count == 0 && position6Count == 0) {
        //    return 4;
        //}

        if (position6Count > position5Count && position6Count > position4Count) {
            return 6;
        } else if (position5Count > position4Count) {
            return 5;
        }
        return 4;
    }

    private final int HOW_MANY_READINGS = 3;

    public double getDistanceLeft() {

        if (!opModeIsActive()) return 0.0;

        double sum = 0;
        int goodReadings = 0;

        while(opModeIsActive() && goodReadings < HOW_MANY_READINGS) {
            double distance = robot.distanceSensorLeft.getDistance(DistanceUnit.INCH);
            if (distance < 255 && distance > 0) {
                sum += distance;
                goodReadings += 1;
            }
        }
        return sum / HOW_MANY_READINGS;
    }

    public double getDistanceCenter() {

        if (!opModeIsActive()) return 0.0;

        double sum = 0;
        int goodReadings = 0;
        int count = 0;

        while(opModeIsActive() && goodReadings < HOW_MANY_READINGS) {
            count++;
            double distance = robot.distanceSensorCenter.getDistance(DistanceUnit.INCH);
            System.out.println("DISTANCE: " + distance);
            if (distance < 255 && distance > 0) {
                sum += distance;
                goodReadings += 1;
            }
        }
        System.out.println("DISTANCE: " + count + " readings");
        return sum / HOW_MANY_READINGS;
    }

    public double getDistanceLR() {

        if (!opModeIsActive()) return 0.0;

        int goodLeftReadings = 0;
        int goodRightReadings = 0;

        double leftSum = 0.0;
        double rightSum = 0.0;

        while (opModeIsActive()
                && (goodLeftReadings < HOW_MANY_READINGS || goodRightReadings < HOW_MANY_READINGS)) {

            double leftDistance = robot.distanceSensorLeft.getDistance(DistanceUnit.INCH);
            double rightDistance = robot.distanceSensorRight.getDistance(DistanceUnit.INCH);

            if (leftDistance < 255 && leftDistance > 0) {
                goodLeftReadings += 1;
                leftSum += leftDistance;
            }

            if (rightDistance < 255 && rightDistance > 0) {
                goodRightReadings += 1;
                rightSum += rightDistance;
            }
        }

        double leftAvg = leftSum / goodLeftReadings;
        double rightAvg = rightSum / goodRightReadings;

        System.out.println("DISTANCE: left:" + leftAvg + ", right:" + rightAvg);

        return Math.max(leftAvg, rightAvg);
    }

    public double getDistanceLC() {

        if (!opModeIsActive()) return 0.0;

        int goodLeftReadings = 0;
        int goodCenterReadings = 0;

        double leftSum = 0.0;
        double centerSum = 0.0;

        while (opModeIsActive()
                && (goodLeftReadings < HOW_MANY_READINGS || goodCenterReadings < HOW_MANY_READINGS)) {

            double leftDistance = robot.distanceSensorLeft.getDistance(DistanceUnit.INCH);
            double centerDistance = robot.distanceSensorCenter.getDistance(DistanceUnit.INCH);

            if (leftDistance < 255 && leftDistance > 0) {
                goodLeftReadings += 1;
                leftSum += leftDistance;
            }

            if (centerDistance < 255 && centerDistance > 0) {
                goodCenterReadings += 1;
                centerSum += centerDistance;
            }
        }

        double leftAvg = leftSum / goodLeftReadings;
        double centerAvg = centerSum / goodCenterReadings;

        System.out.println("DISTANCE: left:" + leftAvg + ", center:" + centerAvg);

        return Math.max(leftAvg, centerAvg);
    }

    public double getDistanceRC() {

        if (!opModeIsActive()) return 0.0;

        int goodRightReadings = 0;
        int goodCenterReadings = 0;

        double rightSum = 0.0;
        double centerSum = 0.0;

        while (opModeIsActive()
                && (goodRightReadings < HOW_MANY_READINGS || goodCenterReadings < HOW_MANY_READINGS)) {

            double rightDistance = robot.distanceSensorRight.getDistance(DistanceUnit.INCH);
            double centerDistance = robot.distanceSensorCenter.getDistance(DistanceUnit.INCH);

            if (rightDistance < 255 && rightDistance > 0) {
                goodRightReadings += 1;
                rightSum += rightDistance;
            }

            if (centerDistance < 255 && centerDistance > 0) {
                goodCenterReadings += 1;
                centerSum += centerDistance;
            }
        }

        double rightAvg = rightSum / goodRightReadings;
        double centerAvg = centerSum / goodCenterReadings;

        System.out.println("DISTANCE: right:" + rightAvg + ", center:" + centerAvg);

        return Math.max(rightAvg, centerAvg);
    }

}
