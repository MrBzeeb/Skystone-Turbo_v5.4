package org.firstinspires.ftc.aperture.nessie.autonomous.red;

/**
 * RedThreeStonesAuto
 *
 * @author Aperture Science
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.aperture.libraries.PIDController;
import org.firstinspires.ftc.aperture.nessie.autonomous.AutoCommon;

@Autonomous(name="RED Three Stones Auto", group="RED")
@Disabled
public class RedThreeStonesAuto extends AutoCommon {
    private final double FACING_FORWARD = 0;
    private final double FACING_RIGHT = -90;

    @Override
    public void runOpMode() {

        // optimized for turns at 1.0 speed
        PIDController pidRotate90 = new PIDController(.01, .00012, 0);
        PIDController pidRotate60 = new PIDController(.01, .00012, 0);

        super.runOpMode();

        int whichPosition = findSkystoneRedBitmap();
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
        driveGyro(getDistanceCenter() + 2, 0.2, FACING_FORWARD);
        if (!opModeIsActive()) return;

        // Grab the skystone
        robot.servoGrip.setPosition(robot.GRIP_CLOSE);
        sleep(300);

        // Lift up the block
        driveGyroAndSetScissorHeightUp(-7, 0.3, FACING_FORWARD, 400);

        // Turn to the far wall
        rotate(-90 + 3, 1.0, pidRotate90);
        if (!opModeIsActive()) return;

        // Extend the 4 Bar and Skystone
        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_DOWN);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Drive toward the far wall
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if (whichPosition == 4) {
            driveGyro(55,0.8, FACING_RIGHT);
        }

        if(whichPosition == 5) {
            driveGyro(47,0.8, FACING_RIGHT);
        }

        if (whichPosition == 6) {
            driveGyro(39, 0.8, FACING_RIGHT);
        }

        if (!opModeIsActive()) return;

        driveGyroAndSetScissorHeightUp(7.0,0.3, FACING_RIGHT, 475);

        sleep(300);
        double distance = getDistanceCenter();
        distance = Range.clip(distance, 0, 30.0);
        driveGyro(distance, 0.3, FACING_RIGHT);
        if (!opModeIsActive()) return;

        robot.servoGrip.setPosition(robot.GRIP_OPEN);
        sleep(100);

        // Move the 4 Bar to intake position
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);

        // Drive back toward the middle line
        driveGyroAndSetScissorHeightDown(-10, 0.5, FACING_RIGHT, 0);

        // Drive back to front wall
        driveGyro(-83,0.8, FACING_RIGHT + 1);
        if (!opModeIsActive()) return;

        // Drive into the wall
        if (whichPosition == 5 || whichPosition == 6) {
            driveGyro(-15,0.3, FACING_RIGHT);
        }

        // Otherwise, turn and strafe into the wall
        if (whichPosition == 4) {
            driveGyro(-2,0.3, FACING_RIGHT);
            sleep(300);
            rotate(90, 1.0, pidRotate90);
            strafe(0.4, 10);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Drive away from the wall
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if (whichPosition == 4) {
            driveGyro(getDistanceLeft(), 0.4, FACING_FORWARD);
            robot.resetDriveEncoders();
            robot.motorFrontLeft.setPower(0);
            robot.motorBackLeft.setPower(-0.2);
            robot.motorBackRight.setPower(0.4);
            robot.motorFrontRight.setPower(0.2);

            while(robot.motorFrontRight.getCurrentPosition() < 400 && opModeIsActive()) {
                // Do nothing
            }
            robot.stop();
            sleep(  100);
        }

        if (whichPosition == 5) {
            driveGyro(1.0, 0.4, FACING_RIGHT);
            rotate(90,1.0, pidRotate90);
            driveGyro(getDistanceLeft(), 0.2, FACING_FORWARD);
        }

        if (whichPosition == 6) {
            driveGyro(8.5, 0.4, FACING_RIGHT);
            rotate(90,1.0, pidRotate90);
            driveGyro(getDistanceLeft(), 0.2, FACING_FORWARD);
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
        sleep(100);

        // Pull the block off the floor
        driveGyroAndSetScissorHeightUp(-4, 0.4, FACING_FORWARD, 400);
        sleep(100);

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

        driveGyroAndSetScissorHeightUp(7.0,0.3, FACING_RIGHT, 475);

        sleep(300);
        distance = getDistanceCenter();
        distance = Range.clip(distance, 0, 30.0);
        driveGyro(distance, 0.3, FACING_RIGHT);
        if (!opModeIsActive()) return;

        robot.servoGrip.setPosition(robot.GRIP_OPEN);
        sleep(100);

        // Move the 4 Bar back to intake position
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);

        // drive back toward the middle line
        driveGyroAndSetScissorHeightDown(-10, 0.5, FACING_RIGHT, 0);

        // drive back to next stone
        if (whichPosition == 6) {
            driveGyro(-20, 0.5, FACING_RIGHT + 1);
        } else {
            driveGyro(-12, 0.4, FACING_RIGHT + 1 );
        }
        rotate(90, 1.0, pidRotate90);
        driveGyro(getDistanceCenter() + 2, 0.2, FACING_FORWARD);
        if (!opModeIsActive()) return;

        // grab the stone
        robot.servoGrip.setPosition(robot.GRIP_CLOSE);
        sleep(300);

        // lift up the block
        driveGyroAndSetScissorHeightUp(-7, 0.3, FACING_FORWARD, 400);

        // turn to the far wall
        rotate(-90 + 3, 1.0, pidRotate90);
        if (!opModeIsActive()) return;

        // Raise the 4 Bar to drop the Skystone
        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_UP);

        if (whichPosition == 6) {
            driveGyro(16, 0.5, FACING_RIGHT + 1);
        } else {
            driveGyro(8, 0.4, FACING_RIGHT + 1 );
        }

        if (!opModeIsActive()) return;

        driveGyroAndSetScissorHeightUp(7.0,0.3, FACING_RIGHT, 475);

        sleep(300);
        distance = getDistanceCenter();
        distance = Range.clip(distance, 0, 30.0);
        driveGyro(distance, 0.3, FACING_RIGHT);
        if (!opModeIsActive()) return;

        robot.servoGrip.setPosition(robot.GRIP_OPEN);
        sleep(100);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Head back to the middle line
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        driveGyroAndSetScissorHeightDown(-10, 0.8, FACING_RIGHT + 2, 0);

        // Move the 4 Bar back to intake position
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);

        if (runTimer.milliseconds() > 29500) return;

        driveGyro(-28,0.8, FACING_RIGHT + 2);


    }

}
