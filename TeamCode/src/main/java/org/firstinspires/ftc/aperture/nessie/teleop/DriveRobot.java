package org.firstinspires.ftc.aperture.nessie.teleop;

/**
 * TeleOp
 *
 * @author Aperture Science
 */

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.aperture.nessie.RobotHardware;

/**
 * TODO  automatically remember next brick placement height
 */

@TeleOp(name="Drive Nessie", group="TeleOp")
public class DriveRobot extends LinearOpMode {

    private RobotHardware robot;

    private ElapsedTime timerLED;
    private ElapsedTime gripTimer;

    private int colorIndex = 0;
    private int[] colors =
            {
                    Color.parseColor("blue"),
                    Color.rgb(255, 37, 0)  // orange
            };

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap, true);

        telemetry.addData("READY", "robot done initializing.");
        telemetry.update();

        waitForStart();

        // set servo positions
        robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_UP);
        robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_UP);
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);
        robot.servoGrip.setPosition(robot.GRIP_FULL_OPEN);
        robot.servoCapGripper.setPosition(robot.CAP_GRIPPER_CLOSED);
        robot.servoCap.setPosition(robot.CAP_IN_POSITION);


        timerLED = new ElapsedTime();
        timerLED.reset();

        gripTimer = new ElapsedTime();

        // the main loop
        while (opModeIsActive()) {

            stickDriving();
            control4Bar();
            controlScissor();
            controlIntake();
            controlWaffle();
            controlCap();
            controlLEDs();

            telemetry.update();
        }

    }

    /*************************************************************
     * controlWaffle
     *
     * GamePad1 x --> waffle grabbers up/down
     *************************************************************/
    private boolean wasXPressed = false;
    private double waffleRightPos = robot.RIGHT_WAFFLE_UP;
    private double waffleLeftPos = robot.LEFT_WAFFLE_UP;

    private void controlWaffle() {

        if (gamepad1.x && !wasXPressed) {

            if (waffleRightPos == robot.RIGHT_WAFFLE_UP) {
                waffleRightPos = robot.RIGHT_WAFFLE_DOWN;
                waffleLeftPos = robot.LEFT_WAFFLE_DOWN;
            } else {
                waffleRightPos = robot.RIGHT_WAFFLE_UP;
                waffleLeftPos = robot.LEFT_WAFFLE_UP;
            }
        }
        wasXPressed = gamepad1.x;

        robot.servoWaffleR.setPosition(waffleRightPos);
        robot.servoWaffleL.setPosition(waffleLeftPos);
    }

    /*************************************************************
     * controlCap
     *
     * Control the capstone delivery.
     *      GamePad 1 back --> toggle delivery/return
     *************************************************************/
    private boolean wasYPressed = false;
    private boolean wasLBPressed = false;
    private double capPosition = robot.CAP_IN_POSITION;
    private double capGripperPosition = robot.CAP_GRIPPER_CLOSED;

    public void controlCap() {

        if (gamepad1.dpad_up && !wasDpadUpPressed) {

            if (capPosition == robot.CAP_IN_POSITION) {
                gripPosition = 0.57;
                if (barPosition < 0.6) {
                    barPosition = 0.1;
                } else {
                    barPosition = robot.FOUR_BAR_INIT;
                }
                capPosition = robot.CAP_OUT_POSITION;
                waffleRightPos = robot.RIGHT_WAFFLE_UP;
                waffleLeftPos = robot.LEFT_WAFFLE_UP;
            } else {
                capPosition = robot.CAP_IN_POSITION;
            }
        }
        wasDpadUpPressed = gamepad1.dpad_up;

        if(gamepad1.left_bumper && !wasLBPressed) {

            System.out.println("LEFT BUMPER PRESSED!");
            System.out.println("LEFT BUMPER: " + capGripperPosition);
            System.out.println("LEFT BUMPER: " + capPosition);

            if (capGripperPosition == robot.CAP_GRIPPER_CLOSED && capPosition == robot.CAP_OUT_POSITION) {
                capGripperPosition = robot.CAP_GRIPPER_OPEN;
            }
        }
        wasLBPressed = gamepad1.left_bumper;

        robot.servoCap.setPosition(capPosition);
        robot.servoCapGripper.setPosition(capGripperPosition);
    }

    /*************************************************************
     * control4Bar
     *
     * GamePad1  -->
     *************************************************************/
    private final int GRIP_TIME_DELAY = 500;
    private double barPosition = robot.FOUR_BAR_INTAKE;
    private boolean isGripNow = false;
    private final double BAR_DELTA = 0.01;
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;

    private void control4Bar() {

        // auto lift the 4 bar on a grip
        if (isGripNow && gripTimer.milliseconds() > GRIP_TIME_DELAY) {
            if (barPosition > robot.FOUR_BAR_DELIVERY_UP) {
                barPosition = robot.FOUR_BAR_DELIVERY_DOWN;
            }
            isGripNow = false;
        }

        // manual control of the 4 bar
        if (gamepad1.dpad_right) {
            barPosition = Range.clip(barPosition + BAR_DELTA, 0.1, robot.FOUR_BAR_INTAKE);
        }
        if (gamepad1.dpad_left) {
            barPosition = Range.clip(barPosition - BAR_DELTA, 0.1, robot.FOUR_BAR_INTAKE);
        }

        if (gamepad1.y && !wasYPressed) {
            barPosition = robot.FOUR_BAR_DELIVERY_UP;
        }
        /*
        if (gamepad1.dpad_down && !wasDpadDownPressed) {
            barPosition = robot.FOUR_BAR_DELIVERY_DOWN;
        }
        */
        wasYPressed = gamepad1.y;
        //wasDpadDownPressed = gamepad1.dpad_down;


        double delta = 0.0;
        if (robot.motorScissorR.getCurrentPosition() > 2500 && barPosition == robot.FOUR_BAR_DELIVERY_DOWN) {
            int diff = robot.motorScissorR.getCurrentPosition() - 2500;
            delta = (diff / 1000.0) * 0.04;

            if (barPosition == robot.FOUR_BAR_DELIVERY_UP) {
                delta = -delta;
            }
        }

        robot.servo4Bar.setPosition(barPosition + delta);
        telemetry.addData("delta", delta);
        telemetry.addData("barPosition (+delta)", barPosition + delta);
    }

    /*************************************************************
     * controlIntake
     *
     * Control the stone gripper and slider.
     *      GamePad 1 a --> open/close gripper
     *
     *      GamePad 2 DPad Right --> slider goes out
     *      GamePad 2 DPad Left  --> slider goes in
     *************************************************************/
    private boolean wasAPressed = false;
    private double gripPosition = robot.GRIP_FULL_OPEN;

    private void controlIntake() {

        if (isStopRequested()) return;  // do not remove this

        if (gamepad1.a && !wasAPressed) {
            if (gripPosition == robot.GRIP_FULL_OPEN) {
                gripPosition = robot.GRIP_CLOSE;

                // auto lift the block using only the 4 bar
                gripTimer.reset();
                isGripNow = true;
            }
            else {
                 isGripNow = false;
                //barPosition = robot.FOUR_BAR_INTAKE;
                gripPosition = robot.GRIP_FULL_OPEN;
            }
        }
        robot.servoGrip.setPosition(gripPosition);
        wasAPressed = gamepad1.a;
    }



    /*************************************************************
     * controlScissor
     *
     * Control the height of the scissor lift.
     *      GamePad 1 DPad Down --> scissor lift down
     *      GamePad 1 DPad Up   --> scissor lift up
     *************************************************************/
    private double scissor_speed = 0.35;
    private final double SCISSOR_UP_SPEED = 0.8;
    private final double SCISSOR_DOWN_SPEED = 0.8;
    private final double SCISSOR_DOWN_SLOW_SPEED = 0.15;

    private boolean isAutoScissorDown = false;
    private boolean wasBPressed = false;

    private void controlScissor() {

        // overrides
        if (gamepad1.dpad_down && gamepad1.right_bumper) {
            if (robot.touchScissorR.getState()) {
                robot.motorScissorR.setPower(scissor_speed / 2);
            } else {
                robot.motorScissorR.setPower(0.0);
            }

            if (robot.touchScissorL.getState()) {
                robot.motorScissorL.setPower(scissor_speed / 2);
            } else {
                robot.motorScissorL.setPower(0.0);
            }

            isAutoScissorDown = false;
            return;
        }

        // auto drop the scissor
        if (gamepad1.b && !wasBPressed) {
            isAutoScissorDown = true;
        }
        wasBPressed = gamepad1.b;

        // pull the scissor down slowly
        if (gamepad1.right_bumper) {

            // set the speed on the way down
            scissor_speed = -SCISSOR_DOWN_SLOW_SPEED;

            // if touch sensed --> reset encoder
            if (!robot.touchScissorR.getState()
                    || robot.motorScissorR.getCurrentPosition() < robot.SCISSOR_MIN_CUTOFF) {
                robot.motorScissorR.setPower(0.0);
                robot.resetScissorREncoder();
            } else {
                robot.motorScissorR.setPower(scissor_speed);
            }

            if (!robot.touchScissorL.getState()
                    || robot.motorScissorL.getCurrentPosition() < robot.SCISSOR_MIN_CUTOFF) {
                robot.motorScissorL.setPower(0.0);
                robot.resetScissorLEncoder();
            } else {
                robot.motorScissorL.setPower(scissor_speed);
            }

            isAutoScissorDown = false;

        } else if (gamepad1.right_trigger > robot.TRIGGER_THRESHOLD) {  // trying to go up

            // set the speed on the way up
            scissor_speed = SCISSOR_UP_SPEED;
            if (robot.motorScissorR.getCurrentPosition() > robot.SCISSOR_MAX_POSITION - 400) {
                scissor_speed /= 2;
            }

            // right motor
            if (robot.motorScissorR.getCurrentPosition() < robot.SCISSOR_MAX_POSITION) {
                robot.motorScissorR.setPower(scissor_speed);
            } else {
                robot.motorScissorR.setPower(0.0);
            }

            // left motor
            if (robot.motorScissorL.getCurrentPosition() < robot.SCISSOR_MAX_POSITION) {
                robot.motorScissorL.setPower(scissor_speed);
            } else {
                robot.motorScissorL.setPower(0.0);
            }

            isAutoScissorDown = false;

        } else {
            if (isAutoScissorDown) {
                setScissorHeightDown(0);
                barPosition = robot.FOUR_BAR_INTAKE;
            } else {
                robot.motorScissorR.setPower(0.0);
                robot.motorScissorL.setPower(0.0);
                isAutoScissorDown = false;
            }
        }

        telemetry.addData("touchScissorR", robot.touchScissorR.getState());
        telemetry.addData("motorScissorR", robot.motorScissorR.getCurrentPosition());
        telemetry.addData("touchScissorL", robot.touchScissorL.getState());
        telemetry.addData("motorScissorL", robot.motorScissorL.getCurrentPosition());
        telemetry.addData("scissor_speed", scissor_speed);
    }

    boolean isAutoScissorUp = false;
    public void setScissorHeightUp(double targetHeight) {
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
            isAutoScissorUp = false;
        }
    }

    public void setScissorHeightDown(double targetHeight) {

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
            isAutoScissorDown = false;
        }
    }

    /*************************************************************
     * stickDriving
     *
     * Drive the robot using the sticks.
     *      GamePad 1 Left Stick   --> forward/backward/strafe
     *      GamePad 1 Right Stick  --> turn
     *      GamePad 1 Left Bumper  --> fast driving (must hold)
     *      GamePad 1 Left Trigger --> slow driving (must hold)
     *************************************************************/
    private double speedModifier = robot.DRIVE_SPEED_MED;

    private void stickDriving() {

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        speedModifier = robot.DRIVE_SPEED_MED;

        /*
        if (gamepad1.left_bumper) {
            speedModifier = robot.DRIVE_SPEED_FAST;
        }
        */

        if (gamepad1.left_trigger > robot.TRIGGER_THRESHOLD     // manual control
                || waffleRightPos == robot.RIGHT_WAFFLE_DOWN)   // waffle gripper is down
        {
            speedModifier = robot.DRIVE_SPEED_SLOW;
        }

        robot.startMove(drive, strafe, turn, speedModifier);
    }

    public void controlLEDs() {

        // cycle through colors
        if (timerLED.milliseconds() > 1000) {
            colorIndex = (colorIndex + 1) % colors.length;
            timerLED.reset();
        }

        robot.ledStrip.setBrightness((int)(Math.random() * 32));
        robot.ledStrip.setColor(colors[colorIndex]);
    }

}