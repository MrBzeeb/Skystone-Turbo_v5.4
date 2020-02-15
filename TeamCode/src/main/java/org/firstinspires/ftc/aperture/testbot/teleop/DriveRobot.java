package org.firstinspires.ftc.aperture.testbot.teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.aperture.testbot.RobotHardware;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TestBot TeleOp", group="TestBot")
@Disabled
public class DriveRobot extends LinearOpMode {

    private RobotHardware robot;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);

        waitForStart();

        timer = new ElapsedTime();
        while (opModeIsActive()) {

            stickDriving();
            controlLEDs();

            telemetry.update();
        }

    }

    private int colorIndex = 0;
    private int[] colors =
                {
                    Color.parseColor("blue"),
                    Color.rgb(255, 37, 0)
                };


    public void controlLEDs() {

        // cycle through colors
        if (timer.milliseconds() > 1000) {
            colorIndex = (colorIndex + 1) % colors.length;
            timer.reset();
        }

        robot.ledStrip.setColor(colors[colorIndex]);
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Drive the robot.
    //   + GamePad 1 Left Stick   --> forward/backward/strafe
    //   + GamePad 1 Right Stick  --> turn
    //   + GamePad 1 Left Bumper  --> fast driving
    //   + GamePad 1 Left Trigger --> slow driving
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    private void stickDriving() {

        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        robot.startMove(drive, strafe, turn);

        // debugging
        telemetry.addData("drive", drive);
        telemetry.addData("strafe", strafe);
        telemetry.addData("turn", turn);


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

        telemetry.addData("frontLeftPower", Range.clip(frontLeftPower, -1, 1));
        telemetry.addData("backLeftPower", Range.clip(backLeftPower, -1, 1));
        telemetry.addData("frontRightPower", Range.clip(frontRightPower, -1, 1));
        telemetry.addData("backRightPower", Range.clip(backRightPower, -1, 1));

        telemetry.addData("LeftStick.y", gamepad1.left_stick_y);
        telemetry.addData("LeftStick.x", gamepad1.left_stick_x);
    }

}