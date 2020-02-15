package org.firstinspires.ftc.aperture.nessie.autonomous.blue;

/**
 * BlueThreeStonesAuto
 *
 * @author Aperture Science
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.aperture.libraries.PIDController;
import org.firstinspires.ftc.aperture.nessie.autonomous.AutoCommon;

@Autonomous(name="BLUE Many Stones Auto", group="BLUE")
@Disabled
public class BlueThreeStonesAuto extends AutoCommon {

    private final double FACING_FORWARD = 0;
    private final double FACING_LEFT = 90;

    @Override
    public void runOpMode() {

        // optimized for turns at 1.0 speed
        PIDController pidRotate90 = new PIDController(.01, .00012, 0);
        PIDController pidRotate60 = new PIDController(.01, .00012, 0);

        super.runOpMode();

        // where is the Skystone?
        int whichPosition = findSkystoneBlueBitmap();

        telemetry.addData("Skystone at position ", whichPosition);
        telemetry.update();

        robot.ledStrip.turnAllOff();

        // pop out the slider once we start
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
            strafe(0.4, -12);
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
        driveGyro(-6, 0.3, FACING_FORWARD);

        // turn to the far wall
        rotate(90 - 3, 1.0, pidRotate90);
        if (!opModeIsActive()) return;

        // push out the slider and skystone
        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_DOWN);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Drive toward the far wall
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if (whichPosition == 4) {
            driveGyro(55, 0.8, FACING_LEFT);
        }

        if (whichPosition == 5) {
            driveGyro(47, 0.8, FACING_LEFT);
        }

        if (whichPosition == 6) {
            driveGyro(39, 0.8, FACING_LEFT);
        }

        if (!opModeIsActive()) return;

        driveGyro(7.0,0.3, FACING_LEFT);

        sleep(300);
        double distance = getDistanceCenter();
        distance = Range.clip(distance, 0, 30.0);
        driveGyro(distance, 0.3, FACING_LEFT);
        if (!opModeIsActive()) return;

        robot.servoGrip.setPosition(robot.GRIP_OPEN);
        sleep(100);

        // pull the slider back in
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);

        // drive back toward the middle line
        driveGyro(-10, 0.5, FACING_LEFT);

        // drive back to front wall
        driveGyro(-83,0.8, FACING_LEFT + 1);
        if (!opModeIsActive()) return;

        // drive into the wall
        if (whichPosition == 5 || whichPosition == 6) {
            driveGyro(-15,0.3, FACING_LEFT);
        }

        // otherwise, turn and strafe into the wall
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
            robot.motorFrontLeft.setPower(0);
            robot.motorBackLeft.setPower(-0.2);
            robot.motorBackRight.setPower(0.4);
            robot.motorFrontRight.setPower(0.2);

            while(robot.motorFrontRight.getCurrentPosition() < 400 && opModeIsActive()) {
                // do nothing
            }
            robot.stop();
            sleep(  100);
        }

        if (whichPosition == 5) {
            driveGyro(1.0, 0.4, FACING_LEFT);
            rotate(-90,1.0, pidRotate90);
            driveGyro(getDistanceLeft(), 0.2, FACING_FORWARD);
        }

        if (whichPosition == 6) {
            driveGyro(8.5, 0.4, FACING_LEFT);
            rotate(-90,1.0, pidRotate90);
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

        // pull the block off the floor
        driveGyro(-4, 0.4, FACING_FORWARD);
        sleep(100);

        if (whichPosition == 5 || whichPosition == 6) {
            rotate(90, 1.0, pidRotate90);
        }

        if (whichPosition == 4) {
            rotate(90, 0.7, pidRotate90);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Deliver second skystone
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if (whichPosition == 4) {
            driveGyro(79,0.8, FACING_LEFT);
        }

        if(whichPosition == 5) {
            driveGyro(78,0.8, FACING_LEFT);
        }

        if (whichPosition == 6) {
            driveGyro(71, 0.8, FACING_LEFT);
        }

        if (!opModeIsActive()) return;

        driveGyro(7.0,0.3, FACING_LEFT);

        sleep(300);
        distance = getDistanceCenter();
        distance = Range.clip(distance, 0, 30.0);
        driveGyro(distance, 0.3, FACING_LEFT);
        if (!opModeIsActive()) return;

        robot.servoGrip.setPosition(robot.GRIP_OPEN);
        sleep(100);

        // pull the slider back in
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);

        // drive back toward the middle line
        driveGyro(-10, 0.5, FACING_LEFT);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //Retrieve the Third Stone
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // drive back to next stone
        if (whichPosition == 6) {
            driveGyro(-20, 0.5, FACING_LEFT + 1);
        } else {
            driveGyro(-12, 0.4, FACING_LEFT + 1 );
        }
        rotate(-90, 1.0, pidRotate90);
        driveGyro(getDistanceCenter() + 2, 0.2, FACING_FORWARD);
        if (!opModeIsActive()) return;

        // grab the stone
        robot.servoGrip.setPosition(robot.GRIP_CLOSE);
        sleep(300);

        // lift up the block
        driveGyro(-7, 0.3, FACING_FORWARD);

        // turn to the far wall
        rotate(90 + 3, 1.0, pidRotate90);
        if (!opModeIsActive()) return;

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Drive toward Far Wall
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // push out the slider and skystone
        robot.servo4Bar.setPosition(robot.FOUR_BAR_DELIVERY_UP);

        if (whichPosition == 6) {
            driveGyro(16, 0.5, FACING_LEFT + 1);
        } else {
            driveGyro(8, 0.4, FACING_LEFT + 1 );
        }

        if (!opModeIsActive()) return;

        driveGyro(7.0,0.3, FACING_LEFT);

        sleep(300);
        distance = getDistanceCenter();
        distance = Range.clip(distance, 0, 30.0);
        driveGyro(distance, 0.3, FACING_LEFT);
        if (!opModeIsActive()) return;

        // Drops Third Stone
        robot.servoGrip.setPosition(robot.GRIP_OPEN);
        sleep(100);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Head back to the middle line
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        driveGyro(-10, 0.8, FACING_LEFT);

        // push the slider back in for normal intake
        robot.servo4Bar.setPosition(robot.FOUR_BAR_INTAKE);

        if (runTimer.milliseconds() > 29500) return;

        driveGyro(-28,0.8, FACING_LEFT + 2);


    }
}