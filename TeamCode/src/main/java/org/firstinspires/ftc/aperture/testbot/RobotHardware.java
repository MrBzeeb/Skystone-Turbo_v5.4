package org.firstinspires.ftc.aperture.testbot;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.aperture.libraries.QwiicLEDStrip;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class RobotHardware {

    public static final double DRIVE_SPEED_SLOW = 0.2;
    public static final double DRIVE_SPEED_MED = 0.7;
    public static final double DRIVE_SPEED_FAST = 1.0;

    public static final double TRIGGER_THRESHOLD = 0.2;
    public static final double JOYSTICK_THRESHOLD = 0.1;

    public static final double ENCODER_TICKS_20 = 537.6;

    public static final double WHEEL_DIAMETER = 4;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double ROBOT_WIDTH = 14.5;
    public static final double TURN_CIRCUMFERENCE = ROBOT_WIDTH * Math.PI * 2;

    public BNO055IMU imu;
    public boolean calibratedIMU;
    public QwiicLEDStrip ledStrip;
    public final int ORANGE = Color.rgb(255, 37, 0);

    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    private HardwareMap hwMap = null;

    public RobotHardware(HardwareMap hardwareMap)  {

        hwMap = hardwareMap;

        // setup the motors
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
        resetDriveEncoders();

        // setup the LED strips
        ledStrip = hardwareMap.get(QwiicLEDStrip.class, "ledStrip");
        ledStrip.setBrightness(5);
    }

    public void stop() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
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

    public void startMove(double drive, double strafe, double turn) {
        double frontLeftPower  = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower   = drive - strafe + turn;
        double backRightPower  = drive + strafe - turn;

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

    public double convertInchesToTicks(double inches) {
        return inches/WHEEL_CIRCUMFERENCE * ENCODER_TICKS_20;
    }

    public double convertDegreesToTicks(double degrees) {
        return (((degrees / 360.0) * TURN_CIRCUMFERENCE) / WHEEL_CIRCUMFERENCE) * ENCODER_TICKS_20;
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

    public void initializeIMU() {
        //------------------------------------------------------------
        // IMU - BNO055
        // Set up the parameters with which we will use our IMU.
        // + 9 degrees of freedom
        // + use of calibration file (see calibration program)
        //------------------------------------------------------------
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitImuCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.mode                = BNO055IMU.SensorMode.NDOF;

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

    public void startMotorL(double power) {
        motorFrontLeft.setPower(Range.clip(power, -1.0, 1.0));
        motorBackLeft.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void startMotorR(double power) {
        motorFrontRight.setPower(Range.clip(power, -1.0, 1.0));
        motorBackRight.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void startStrafe(double power) {
        motorFrontLeft.setPower(Range.clip(power, -1.0, 1.0));
        motorBackLeft.setPower(Range.clip(-power, -1.0, 1.0));
        motorFrontRight.setPower(Range.clip(-power, -1.0, 1.0));
        motorBackRight.setPower(Range.clip(power, -1.0, 1.0));
    }

}