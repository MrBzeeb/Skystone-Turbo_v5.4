package org.firstinspires.ftc.aperture.nessie.autonomous.red;

/**
 * RedStonesAndWaffleAuto
 *
 * @author Aperture Science
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.aperture.libraries.PIDController;
import org.firstinspires.ftc.aperture.nessie.autonomous.AutoCommon;

@Autonomous(name="RED Stones/Waffle Auto", group="RED")
public class RedStonesAndWaffleAuto extends AutoCommon {

    private final double FACING_FORWARD = 0;
    private final double FACING_RIGHT = -90;
    private final double FOUR_BAR_FIRST_PEG = 0.66;
    private final double FOUR_BAR_SECOND_PEG = 0.55;

    @Override
    public void runOpMode() {

        // optimized for turns at 1.0 speed
        PIDController pidRotate90 = new PIDController(.01, .00012, 0);
        PIDController pidRotate60 = new PIDController(.01, .00012, 0);

        super.runOpMode();

        int whichPosition = findSkystoneRedBitmap();
        telemetry.addData("Skystone at position ", whichPosition);
        telemetry.update();

        // LEDs on for consistent lighting
        robot.ledStrip.turnAllOff();

        // pop out the 4-bar once we start
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Approach the stones
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        robot.servoGrip.setPosition(robot.GRIP_OPEN);
        driveGyro(20, 0.4, FACING_FORWARD);

        if (isStopRequested()) {
            robot.stop();
            return;
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Align with the skystone
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if (whichPosition == 4) {
            strafe(0.3, 3.5);
        }

        if (whichPosition == 5) {
            strafe(0.3, -4);
        }

        if (whichPosition == 6) {
            strafe(0.3, -13);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Approach and grab the skystone
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        driveGyro(getDistanceCenter() + 1, 0.2, FACING_FORWARD);
        if (!opModeIsActive()) return;

        // grab the skystone
        robot.servoGrip.setPosition(robot.GRIP_CLOSE);
        sleep(200);

        // pull the 4-bar up to lift the block
        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_DOWN);

        // back away from stones
        driveGyro(-6, 0.3, FACING_FORWARD);

        // turn to the far wall
        rotate(-90 + 3, 1.0, pidRotate90);
        if (!opModeIsActive()) return;

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Drive toward the far wall
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if (whichPosition == 4) {
            driveGyro(80, 0.8, FACING_RIGHT);
        }

        if (whichPosition == 5) {
            driveGyro(72.5, 0.8, FACING_RIGHT);
        }

        if (whichPosition == 6) {
            driveGyro(65, 0.8, FACING_RIGHT);
        }

        if (!opModeIsActive()) return;

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Turn, grab, and place the waffle
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_AUTO_FIRST_PEG);
        rotate(90 - 3, 1.0, pidRotate90);

        // approach the waffle
        sleep(300);
        double distance = getDistanceCenter();
        distance = Range.clip(distance,2.0,20.0);
        driveGyro(distance + 1, 0.25, FACING_FORWARD);

        if (!opModeIsActive()) return;

        // grab the waffle
        robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_DOWN);
        robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_DOWN);
        sleep(200);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Move/rotate the waffle into the building site
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        driveGyro(2, 0.25, FACING_FORWARD);
        driveGyro(-9, 0.4, -30);

        if (!opModeIsActive()) return;

        // finish the rotation
        rotate(-60,0.8, pidRotate60);

        // distance check -- did we get the waffle?
        if (getDistanceCenter() > 4) {
            robot.servoGrip.setPosition(robot.GRIP_FULL_OPEN);
            robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);
            robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_UP);
            robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_UP);
            return;
        }

        // shove waffle into the wall
        driveGyro(14, 0.5, FACING_RIGHT);
        if (!opModeIsActive()) return;

        // let go of the waffle
        robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_UP);
        robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_UP);
        sleep(200);

        // drive back a small amount
        driveGyro(-1.5, 0.3, FACING_RIGHT);

        // drop the first stone
        robot.servoGrip.setPosition(robot.GRIP_OPEN);
        sleep(500);

        // back up, then pull the 4-bar back in
        driveGyro(-6, 0.8, FACING_RIGHT);
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);

        // drive back to front wall
        driveGyro(-84,0.8, FACING_RIGHT + 1);
        if (!opModeIsActive()) return;

        // drive into the wall
        if (whichPosition == 5 || whichPosition == 6) {
            driveGyro(-16,0.3, FACING_RIGHT);
        }

        // otherwise, turn and strafe into the wall
        if (whichPosition == 4) {
            driveGyro(-2,0.3, FACING_RIGHT);
            sleep(300);
            rotate(90, 1.0, pidRotate90);
            strafe(0.4, 12);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Drive away from the wall
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if (whichPosition == 4) {
            robot.servo4Bar.setPosition(robot.FOUR_BAR_INIT);
            driveGyro(getDistanceCenter(), 0.4, FACING_FORWARD);
            robot.resetDriveEncoders();
            robot.motorFrontLeft.setPower(0);
            robot.motorBackLeft.setPower(-0.2);
            robot.motorBackRight.setPower(0.4);
            robot.motorFrontRight.setPower(0.2);

            while(robot.motorFrontRight.getCurrentPosition() < 425 && opModeIsActive()) {
                // do nothing
            }
            robot.stop();
            sleep(  100);
        }

        if (whichPosition == 5) {
            driveGyro(4.0, 0.3, FACING_RIGHT);
            sleep(300);
            rotate(90,1.0, pidRotate90);
            strafe(0.3, 1.0);
            driveGyro(getDistanceCenter(), 0.2, FACING_FORWARD);
        }

        if (whichPosition == 6) {
            driveGyro(8.5, 0.4, FACING_RIGHT);
            sleep(200);
            rotate(90,1.0, pidRotate90);
            driveGyro(getDistanceCenter(), 0.2, FACING_FORWARD);
        }

        if (getDistanceCenter() > 20) {
            robot.servoGrip.setPosition(robot.GRIP_FULL_OPEN);
            robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);
            robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_UP);
            robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_UP);
            return;
        }

        if (!opModeIsActive()) return;

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Grab the skystone
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        robot.servoGrip.setPosition(robot.GRIP_CLOSE);
        sleep(250);

        // pull the block off the floor
        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_DOWN);

        // back up
        driveGyro(-4, 0.4, FACING_FORWARD);

        // turn toward the far wall
        if (whichPosition == 5 || whichPosition == 6) {
            rotate(-90, 1.0, pidRotate90);
        }

        if (whichPosition == 4) {
            rotate(-90, 0.7, pidRotate90);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Deliver second skystone
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if (whichPosition == 4) {
            driveGyro(79,0.8, FACING_RIGHT);
        }

        if(whichPosition == 5) {
            driveGyro(78,0.8, FACING_RIGHT);
        }

        if (whichPosition == 6) {
            driveGyro(71, 0.8, FACING_RIGHT);
        }

        if (!opModeIsActive()) return;

        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_AUTO_SECOND_PEG);

        sleep(300);
        distance = getDistanceLeft() + 3;
        distance = Range.clip(distance, 0, 30.0);
        driveGyro(distance, 0.3, FACING_RIGHT);
        if (!opModeIsActive()) return;

        robot.servoGrip.setPosition(robot.GRIP_OPEN);
        sleep(750);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Head back to the middle line
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        if (runTimer.milliseconds() > 29500) return;

        driveGyro(-10,0.8, FACING_RIGHT + 2);
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);
        robot.servoGrip.setPosition(robot.GRIP_FULL_OPEN);
        driveGyro(-28,0.8, FACING_RIGHT + 2);
    }
}