package org.firstinspires.ftc.aperture.nessie.autonomous.blue;

/**
 * BlueStonesAndWaffleAuto
 *
 * @author Aperture Science
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.aperture.libraries.PIDController;
import org.firstinspires.ftc.aperture.nessie.autonomous.AutoCommon;

@Autonomous(name="BLUE Stones/Waffle Auto", group="BLUE")
public class BlueStonesAndWaffleAuto extends AutoCommon {

    private final double FACING_FORWARD = 0;
    private final double FACING_LEFT = 90;

    @Override
    public void runOpMode() {

        // optimized for turns at 1.0 speed
        PIDController pidRotate90 = new PIDController(.01, .00012, 0);
        PIDController pidRotate60 = new PIDController(.01, .00012, 0);
        PIDController pidRotate100 = new PIDController(.01, .00012, 0);

        super.runOpMode();

        // where is the Skystone?
        int whichPosition = findSkystoneBlueBitmap();

        telemetry.addData("Skystone at position ", whichPosition);
        telemetry.update();

        robot.ledStrip.turnAllOff();

        // Prep the 4 Bar at the start
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
            strafe(0.4, -11.5);
        }

        if (whichPosition == 6) {
            strafe(0.3, 4);
        }

        if (whichPosition == 5) {
            strafe(0.3, -3.5);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Approach and grab the skystone
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        driveGyro(getDistanceCenter() + 1, 0.2, FACING_FORWARD);
        if (!opModeIsActive()) return;

        // grab the skystone
        robot.servoGrip.setPosition(robot.GRIP_CLOSE);
        sleep(200);

        // lift up the block and back up
        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_DOWN);
        driveGyro(-6, 0.3, FACING_FORWARD);

        // turn to the far wall
        rotate(90 - 3, 1.0, pidRotate90);
        if (!opModeIsActive()) return;

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Drive toward the far wall
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if (whichPosition == 4) {
            driveGyro(79, 0.8, FACING_LEFT);
        }

        if (whichPosition == 5) {
            driveGyro(74, 0.8, FACING_LEFT);
        }

        if (whichPosition == 6) {
            driveGyro(65, 0.8, FACING_LEFT);
        }

        if (!opModeIsActive()) return;

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Turn, grab, and place the waffle
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_AUTO_FIRST_PEG);
        rotate(-90 + 3, 1.0, pidRotate90);

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
        driveGyro2(-7, 0.4, 30);

        if (!opModeIsActive()) return;

        // finish the rotation
        rotate(60,0.8, pidRotate60);

        // distance check -- did we get the waffle?
        if (getDistanceCenter() > 4) {
            robot.servoGrip.setPosition(robot.GRIP_FULL_OPEN);
            robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);
            robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_UP);
            robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_UP);
            return;
        }

        // shove waffle into the wall
        driveGyro(14, 0.5, FACING_LEFT);
        if (!opModeIsActive()) return;

        // let go of the waffle
        robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_UP);
        robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_UP);

        sleep(200);

        // drive back a small amount
        driveGyro(-1.5, 0.3, FACING_LEFT);

        // drop the first stone
        robot.servoGrip.setPosition(robot.GRIP_OPEN);
        sleep(500);

        // back up, then pull the 4-bar back in
        driveGyro(-6, 0.8, FACING_LEFT);
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);

        // Drive back to front wall
        driveGyro(-87,0.8, FACING_LEFT - 1.5);
        if (!opModeIsActive()) return;

        if (whichPosition == 5 || whichPosition == 6) {
            driveGyro(-15,0.3, FACING_LEFT);
        }

        if (whichPosition == 4) {
            driveGyro(-2,0.3, FACING_LEFT);
            sleep(300);
            rotate(-90, 1.0, pidRotate90);
            strafe(0.4, -10);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Drive away from the wall
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if (whichPosition == 4) {
            driveGyro(getDistanceLeft(), 0.4, FACING_FORWARD);
            robot.resetDriveEncoders();
            robot.motorFrontLeft.setPower(0.2);
            robot.motorBackLeft.setPower(0.4);
            robot.motorBackRight.setPower(-0.2);
            robot.motorFrontRight.setPower(0);

            while(robot.motorFrontLeft.getCurrentPosition() < 400 && opModeIsActive()) {
                //do nothing
            }
            robot.stop();
            sleep(  100);
        }

        if (whichPosition == 5) {
            driveGyro(0.5, 0.3, FACING_LEFT);
            rotate(-90,1.0, pidRotate90);
            strafe(0.3, -2);
            driveGyro(getDistanceCenter() + 2, 0.2, FACING_FORWARD);
        }

        if (whichPosition == 6) {
            driveGyro(5.25, 0.4, FACING_LEFT);
            rotate(-90,1.0, pidRotate90);
            driveGyro(getDistanceCenter() + 2, 0.2, FACING_FORWARD);
        }

        if (getDistanceLeft() > 20) {
            setScissorHeight(0);
            sleep(2000);
            return;
        }

        if (!opModeIsActive()) return;

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Approach and grab the skystone
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        robot.servoGrip.setPosition(robot.GRIP_CLOSE);
        sleep(200);
        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_DOWN);

        driveGyro(-6, 0.4, FACING_FORWARD);
        sleep(100);

        if (whichPosition == 5 || whichPosition == 6) {
            rotate(90, 1.0, pidRotate90);
        }

        if (whichPosition == 4) {
            driveGyro(-12, 0.4, FACING_FORWARD);
            rotate(90, 0.7, pidRotate90);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Deliver second skystone
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if (whichPosition == 4) {
            driveGyro(82,0.8, FACING_LEFT);
        }

        if(whichPosition == 5) {
            driveGyro(80,0.8, FACING_LEFT);
        }

        if (whichPosition == 6) {
            driveGyro(82, 0.8, FACING_LEFT);
        }

        if (!opModeIsActive()) return;

        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_AUTO_SECOND_PEG);
        sleep(700);

        distance = getDistanceLeft() + 3;
        distance = Range.clip(distance, 0, 30.0);
        driveGyro2(distance, 0.3, FACING_LEFT);

        System.out.println("ACTUAL DISTANCE: " + distance);

        if (!opModeIsActive()) return;

        robot.servoGrip.setPosition(robot.GRIP_OPEN);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Head back to the middle line
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        if (runTimer.milliseconds() > 29500) return;

        driveGyro(-10,0.8, FACING_LEFT - 2);
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);
        robot.servoGrip.setPosition(robot.GRIP_FULL_OPEN);
        driveGyro(-28,0.8, FACING_LEFT - 2);
    }
}