package org.firstinspires.ftc.aperture.nessie;

/**
 * Hardware definitions for the competition robot.
 *
 * @author Aperture Science
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.aperture.libraries.QwiicLEDStrip;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Hardware class for the RobotHardware robot.
 */

public class RobotHardware {

    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    public DcMotor motorScissorR;
    public DcMotor motorScissorL;

    public Servo servoGrip;
    public Servo servo4Bar;
    public Servo servoWaffleR;
    public Servo servoWaffleL;
    public Servo servoCap;
    public Servo servoCapGripper;

    public DigitalChannel touchScissorR;
    public DigitalChannel touchScissorL;

    public Rev2mDistanceSensor distanceSensorLeft;
    public Rev2mDistanceSensor distanceSensorCenter;
    public Rev2mDistanceSensor distanceSensorRight;

    public BNO055IMU imu;
    public boolean calibratedIMU;

    public QwiicLEDStrip ledStrip;

    public final static double DRIVE_SPEED_SLOW = 0.3;
    public final static double DRIVE_SPEED_MED = 0.8;
    public final static double DRIVE_SPEED_FAST = 1.0;

    public final static double TRIGGER_THRESHOLD = 0.2;
    public final static double JOYSTICK_THRESHOLD = 0.1;

    public final static double ENCODER_TICKS_20 = 537.6;

    public final static double WHEEL_DIAMETER = 4;
    public final static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public final static double ROBOT_WIDTH = 14.5;
    public final static double TURN_CIRCUMFERENCE = ROBOT_WIDTH * Math.PI * 2;

    public final static double GRIP_INIT = 0.7;
    public final static double GRIP_FULL_OPEN = 0.72;
    public final static double GRIP_OPEN = 0.4;
    public final static double GRIP_CLOSE = 0.25;

    public final static double RIGHT_WAFFLE_UP = 0.55;
    public final static double RIGHT_WAFFLE_DOWN = 0.85;
    public final static double LEFT_WAFFLE_UP = 0.65;
    public final static double LEFT_WAFFLE_DOWN = 0.35;

    public final static int SCISSOR_MAX_POSITION = 3400;
    public final static int SCISSOR_MIN_CUTOFF = 50;
    public final static double SCISSOR_UP_SPEED = 0.6;
    public final static double SCISSOR_DOWN_SPEED = 0.4;

    public final static double CAP_IN_POSITION = 0.40;
    public final static double CAP_OUT_POSITION = 0.87;
    public final static double CAP_GRIPPER_CLOSED = 0.24;
    public final static double CAP_GRIPPER_OPEN = 0.90;

    public final static double FOUR_BAR_INIT = 0.7;                 // 0.68
    public final static double FOUR_BAR_INTAKE = 0.68;              // 0.64
    public final static double FOUR_BAR_DELIVERY_DOWN = 0.66;       // 0.63
    public final static double FOUR_BAR_DELIVERY_UP = 0.1;         // 0.15
    public final static double FOUR_BAR_DELIVERY_AUTO_FIRST_PEG = 0.48;   // 0.46
    public final static double FOUR_BAR_DELIVERY_AUTO_SECOND_PEG = 0.26;   // 0.24

    private HardwareMap hwMap =  null;

    public RobotHardware(HardwareMap ahwMap, boolean isTeleOp) {

        hwMap = ahwMap;

        if (!isTeleOp) initializeIMU();

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Drive Motors
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
        motorBackRight = hwMap.dcMotor.get("motorBackRight");
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
        resetDriveEncoders();

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Prepare other motors
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        motorScissorR = hwMap.dcMotor.get("motorScissorR");
        motorScissorR.setDirection(DcMotor.Direction.REVERSE);
        motorScissorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorScissorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorScissorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorScissorL = hwMap.dcMotor.get("motorScissorL");
        motorScissorL.setDirection(DcMotor.Direction.FORWARD);
        motorScissorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorScissorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorScissorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Setup the servos
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        servoGrip = hwMap.servo.get("servoGrip");
        servo4Bar = hwMap.servo.get("servo4Bar");
        servoWaffleL = hwMap.servo.get("servoWaffleL");
        servoWaffleR = hwMap.servo.get("servoWaffleR");
        servoCap = hwMap.servo.get("servoCap");
        servoCapGripper = hwMap.servo.get("servoCapGripper");

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Setup the sensors
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        touchScissorR = hwMap.digitalChannel.get("touchScissorR");
        touchScissorR.setMode(DigitalChannel.Mode.INPUT);

        touchScissorL = hwMap.digitalChannel.get("touchScissorL");
        touchScissorL.setMode(DigitalChannel.Mode.INPUT);

        distanceSensorLeft = hwMap.get(Rev2mDistanceSensor.class, "distanceSensorLeft");
        distanceSensorCenter = hwMap.get(Rev2mDistanceSensor.class, "distanceSensorCenter");
        distanceSensorRight = hwMap.get(Rev2mDistanceSensor.class, "distanceSensorRight");

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Setup the LEDs
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ledStrip = hwMap.get(QwiicLEDStrip.class, "ledStrip");
    }


    public void initializeIMU() {
        //------------------------------------------------------------
        // IMU - BNO055
        // Set up the parameters with which we will use our IMU.
        // + 9 degrees of freedom
        // + use of calibration file (see calibration program)
        //------------------------------------------------------------
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitImuCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        //parameters.loggingTag          = "IMU";
        //parameters.mode                = BNO055IMU.SensorMode.NDOF;

        parameters.accelerationIntegrationAlgorithm = null;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Read the IMU configuration from the data file saved during calibration.
        // Using try/catch allows us to be specific about the error instead of
        // just showing a NullPointer exception that could come from anywhere in the program.
        calibratedIMU = true;
        try {
            File file = AppUtil.getInstance().getSettingsFile(parameters.calibrationDataFile);
            String strCalibrationData = ReadWriteFile.readFile(file);
            BNO055IMU.CalibrationData calibrationData = BNO055IMU.CalibrationData.deserialize(strCalibrationData);
            imu.writeCalibrationData(calibrationData);
        }
        catch (Exception e) {
            calibratedIMU = false;
        }
    }

    public double convertInchesToTicks(double inches) {
        return inches/WHEEL_CIRCUMFERENCE * ENCODER_TICKS_20;
    }

    public double convertDegreesToTicks(double degrees) {
        return (((degrees / 360.0) * TURN_CIRCUMFERENCE) / WHEEL_CIRCUMFERENCE) * ENCODER_TICKS_20;
    }

    public void resetScissorREncoder() {
        motorScissorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorScissorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetScissorLEncoder() {
        motorScissorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorScissorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetDriveEncoders() {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public void startMotorL(double power) {
        motorFrontLeft.setPower(Range.clip(power, -1.0, 1.0));
        motorBackLeft.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void startMotorR(double power) {
        motorFrontRight.setPower(Range.clip(power, -1.0, 1.0));
        motorBackRight.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void startDrive(double power) {
        motorFrontLeft.setPower(Range.clip(power, -1.0, 1.0));
        motorBackLeft.setPower(Range.clip(power, -1.0, 1.0));
        motorFrontRight.setPower(Range.clip(power, -1.0, 1.0));
        motorBackRight.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void startTurn(double power) {
        motorFrontLeft.setPower(Range.clip(power, -1.0, 1.0));
        motorBackLeft.setPower(Range.clip(power, -1.0, 1.0));
        motorFrontRight.setPower(Range.clip(-power, -1.0, 1.0));
        motorBackRight.setPower(Range.clip(-power, -1.0, 1.0));
    }

    public void startMove(double drive, double strafe, double turn, double modifier) {
        double frontLeftPower  = (drive + strafe + turn) * modifier;
        double frontRightPower = (drive - strafe - turn) * modifier;
        double backLeftPower   = (drive - strafe + turn) * modifier;
        double backRightPower  = (drive + strafe - turn) * modifier;

        double max = Math.max(frontLeftPower, Math.max(frontRightPower,
                Math.max(backLeftPower, backRightPower)));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        motorFrontLeft.setPower(Range.clip(frontLeftPower, -1, 1));
        motorFrontRight.setPower(Range.clip(frontRightPower, -1, 1));
        motorBackLeft.setPower(Range.clip(backLeftPower, -1, 1));
        motorBackRight.setPower(Range.clip(backRightPower, -1, 1));
    }

    /*
     * +power:
     *     ^         |
     *     |         v
     *    [ ]---F---[ ]
     *      |       |
     *      |       |   ===>>
     *      |       |
     *    [ ]-------[ ]
     *     |         ^
     *     v         |
     *
     * -power:
     *         |         ^
     *         v         |
     *        [ ]---F---[ ]
     *          |       |
     *  <<===   |       |
     *          |       |
     *        [ ]-------[ ]
     *         ^         |
     *         |         v
     */
    public void startStrafe(double power) {
        motorFrontLeft.setPower(Range.clip(power, -1.0, 1.0));
        motorBackLeft.setPower(Range.clip(-power, -1.0, 1.0));
        motorFrontRight.setPower(Range.clip(-power, -1.0, 1.0));
        motorBackRight.setPower(Range.clip(power, -1.0, 1.0));
    }

}