package org.firstinspires.ftc.aperture.testbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.aperture.testbot.RobotHardware;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="TestBot Auto", group="TestBot")
@Disabled
public class AutoCommon extends LinearOpMode {

    private RobotHardware robot;
    private double initialHeading;

    /* Tensor Flow */
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/Skystone.tflite"; //For OpenRC, loaded from internal storage to reduce APK size
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AT30c0j/////AAABmQwAYW6b3UgcszoTxj12AutsoJYh6fsfXXDWXjCbXLGDygqdPYi53"
                    +   "2meoJUTsVGKltZN/7fEqRv7Y1DZympsptLtTPyG9s0Xizp6YKafebGW2NALBbxNzQ2jSC"
                    +   "tohvdNCnTMY7ILT9xhcE3u1vLStOBdIy10G9PL0ISDkwv2MzteegEwzIaM4acZs/SF9LR"
                    +   "h4JP8jpPNyfZDUZCTSGp1D9UpagYfaProK/yenpSCuhypkhJ/6s01KFg2oVcfw9ZaQuR9"
                    +   "IfLFrdxXvLrgnrI/QZyhzA/OqQj24BHVT37BQvo8IyNKqZthTGpSziejt+ySvuW5n3AyH"
                    +   "o4wqeCnwYi6mMKrjYGL7sC8NVoV+qmU0CKC";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);
        robot.initializeIMU();

        if (robot.calibratedIMU) {
            telemetry.addData("IMU", "calibrated");
        } else {
            telemetry.addData("IMU", "NOT calibrated");
        }
        telemetry.update();

/*
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
/*
        if (tfod != null) {
            tfod.activate();

        }
*/






        waitForStart();

        // get the initial angles from the IMU
        Orientation angles = robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES);
        initialHeading = angles.firstAngle;

        turnGyro(90);
        sleep(1000);
        turnGyro(-90);
        sleep(1000);
        whipTurn(90);
        sleep(1000);
        whipTurn(-90);


/*
        //turn on flashlight
        CameraDevice.getInstance().setFlashTorchMode(true);

        drive(-10, 0.2);

        int pos6Count = 0;
        int pos5Count = 0;
        int pos4Count = 0;

        elapsedTime.reset();

        while ((pos6Count + pos5Count + pos4Count < 3) && elapsedTime.milliseconds() < 2000) {

            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Skystone")) {
                            double center = (recognition.getBottom() + recognition.getTop()) / 2.0;
                            telemetry.addData(String.format("  center (%d)", i), "%.03f", center);


                            if (center >= 800) {
                                telemetry.addData("skystone", "position 6");
                                pos6Count++;

                            } else if (center >= 600) {
                                telemetry.addData("skystone", "position 5");
                                pos5Count++;
                            } else {
                                telemetry.addData("skystone", "position 4");
                                pos4Count++;
                            }

                        }
                        telemetry.update();
                    }
                }
            }
            sleep(50);
        }

        telemetry.addData("pos6Count", pos6Count);
        telemetry.addData("pos5Count", pos5Count);
        telemetry.addData("pos4Count", pos4Count);
        telemetry.addData("milliseconds", elapsedTime.milliseconds());
        telemetry.update();


        if (pos6Count > pos4Count && pos6Count > pos5Count) {
            whipTurn(20);
            drive(-17, 0.2);
            whipTurn(-20);
        }else if (pos5Count > pos4Count) {
            drive(-15, 0.2);
        }else {
            whipTurn(-20);
            drive(-17,0.2);
            whipTurn(20);

        }


        CameraDevice.getInstance().setFlashTorchMode(false);

        if (tfod != null) {
            tfod.shutdown();
        }


        while (opModeIsActive()) {

        }
*/

        /*
         The Color Sensors read what is in front of it and allow the robot to determine if
         there is a Skystone in front of it or just a Stone.
         */
        /*
        robot.colorSensorL.enableLed(true);
        robot.colorSensorR.enableLed(true);

        drive(32, 0.2);
        while(opModeIsActive()){
            int alphaL = robot.colorSensorL.alpha();
            telemetry.addData("Color Sensor Left", alphaL);

            int alphaR = robot.colorSensorR.alpha();
            telemetry.addData("Color Sensor Right", alphaR);
            telemetry.update();

        }
        robot.colorSensorL.enableLed(false);
        robot.colorSensorR.enableLed(false);
        */


        /*
        whipTurn(90);
        sleep(1000);

        whipTurn(90);
        sleep(1000);

        whipTurn(90);
        sleep(1000);

        whipTurn(90);
        sleep(1000);
        */

    }


    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~
                IMU
       ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    */
    Orientation lastAngles = new Orientation();
    double globalAngle;

    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES);
        globalAngle = 0;
    }

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

        globalAngle -= deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }


    public void drive(double distance, double speed) {
        speed = Math.abs(speed);
        double direction = Math.signum(distance);
        double distanceInTicks = robot.convertInchesToTicks(Math.abs(distance));

        robot.resetDriveEncoders();

        robot.startDrive(speed * direction);
        while (Math.abs(robot.motorBackLeft.getCurrentPosition()) < distanceInTicks && opModeIsActive()) {
        }
        robot.stop();
    }

    public  double getHeading() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle - initialHeading;
    }


    private final double IMU_LAG = 2.7;

    public void whipTurn(double degrees) {
        double speed = 1.0;
        double direction = -Math.signum(degrees);
        double degreesInTicks = robot.convertDegreesToTicks(Math.abs(degrees));

        resetAngle();

        // whip
        robot.resetDriveEncoders();
        robot.startTurn(speed * direction);
        while (Math.abs(robot.motorBackLeft.getCurrentPosition()) < degreesInTicks && opModeIsActive()) {
            // left blank intentionally
        }
        robot.stop();
        sleep(100);

        // correction
        double currentAngle = getAngle();
        double error = Math.abs(currentAngle - degrees) - IMU_LAG;

        speed = 0.2;
        resetAngle();
        robot.startTurn(speed * -direction);
        while (Math.abs(getAngle()) <= error && opModeIsActive()) {
        }
        robot.stop();

        /*
        telemetry.addData("currentAngle", getAngle());
        telemetry.addData("error", Math.abs(getAngle()) - error);
        telemetry.update();
         */


        currentAngle = getAngle();
        error = currentAngle - error;
        direction = Math.signum(error);
        if (error >= 4 ) {
            speed = 0.2;
            resetAngle();
            robot.startTurn(speed * -direction);
            while (Math.abs(getAngle()) <= Math.abs(error) && opModeIsActive()) {
            }
            robot.stop();
        }
    }

    public void turn(double degrees, double speed) {
        speed = Math.abs(speed);
        double direction = Math.signum(degrees);
        double degreesInTicks = robot.convertDegreesToTicks(Math.abs(degrees));

        telemetry.addData("degreesInTicks", degreesInTicks);

        robot.resetDriveEncoders();

        robot.startTurn(speed * direction);
        while (Math.abs(robot.motorBackLeft.getCurrentPosition()) < degreesInTicks && opModeIsActive()) {
        }
        robot.stop();
    }

    public void gradualTurn(double degrees) {
        double speed = 0.25;
        double direction = Math.signum(degrees);
        double degreesInTicks = robot.convertDegreesToTicks(Math.abs(degrees));

        telemetry.addData("degreesInTicks", degreesInTicks);

        robot.resetDriveEncoders();

        robot.startTurn(speed * direction);
        while (Math.abs(robot.motorBackLeft.getCurrentPosition()) < degreesInTicks && opModeIsActive()) {
        }


        robot.stop();
        sleep(100);
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    public int inchesToRotations(double distance) {
        double rotations = distance / robot.WHEEL_CIRCUMFERENCE;
        return (int) (robot.ENCODER_TICKS_20 * rotations);
    }


    public void strafe(double power, double distance) {
        if (isStopRequested()) return;

        int ticks = inchesToRotations(Math.abs(distance));

        double direction = -Math.signum(distance);
        power = Math.abs(power);

        robot.resetDriveEncoders();
        robot.startStrafe(power * direction);
        while (Math.abs(robot.motorBackRight.getCurrentPosition()) < ticks && opModeIsActive()) {
            //robot.startStrafe(power * direction);
        }
        robot.stop();
    }



    private final static double TURNGYRO_SPEED = 0.2;
    private final static double TURNGYRO_LAG   = 0.125;
    public void turnGyro(double degrees) {

        if (isStopRequested()) return;

        degrees *= -1;
        if(degrees == 0) {
            return;
        }
        double direction    = Math.signum(degrees);
        double currHeading  = getHeading();
        double prevHeading  = currHeading;
        double firstHeading = currHeading;
        double tarHeading   = currHeading + degrees;
        while(opModeIsActive() &&  ((direction > 0 && currHeading < tarHeading - TURNGYRO_LAG)
                || (direction < 0 && currHeading > tarHeading + TURNGYRO_LAG))) {
            robot.startMotorL(TURNGYRO_SPEED *  direction);
            robot.startMotorR(TURNGYRO_SPEED * -direction);
            prevHeading = currHeading;
            currHeading = getHeading();
            if(currHeading > prevHeading + 180) currHeading -= 360;
            if(currHeading < prevHeading - 180) currHeading += 360;

            telemetry.addLine().addData("initialHeading:", initialHeading);
            telemetry.addLine().addData("first heading", firstHeading);
            telemetry.addLine().addData(" Target", tarHeading);
            telemetry.addLine().addData("Current", currHeading);
            telemetry.update();
        }
        robot.stop();
    }

}