package org.firstinspires.ftc.aperture.nessie.autonomous.red;

/**
 * RedWaffleRotateAuto
 *
 * @author Aperture Science
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.aperture.libraries.PIDController;
import org.firstinspires.ftc.aperture.nessie.autonomous.AutoCommon;

@Autonomous(name="RED Waffle Rotate Auto", group="RED")
public class RedWaffleRotateAuto extends AutoCommon {

    private final double FACING_FORWARD = 0;
    private final double FACING_RIGHT = -90;

    PIDController pidRotate60 = new PIDController(.01, .00012, 0);

    @Override
    public void runOpMode() {
        super.runOpMode();

        // drive to the waffle
        driveGyro(26, 0.5, FACING_FORWARD);
        strafe(.4,-11);

        // approach the waffle
        sleep(300);
        double distance = getDistanceLeft();
        distance = Range.clip(distance,2.0,20.0);
        driveGyro(distance, 0.25, FACING_FORWARD);

        if (!opModeIsActive()) return;

        // grab the waffle
        robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_DOWN);
        robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_DOWN);
        sleep(200);

        driveGyro(2, 0.25, FACING_FORWARD);
        driveGyro(-9, 0.4, -30);

        if (!opModeIsActive()) return;

        // finish the rotation
        rotate(-60,0.8, pidRotate60);

        // shove waffle into the wall
        if (getDistanceLeft() < 5) {
            driveGyro(12, 0.5, FACING_RIGHT);
        }

        if (!opModeIsActive()) return;

        // release the waffle
        robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_UP);
        robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_UP);
        sleep(200);

        strafe(0.5, -27);
        strafe(0.4, 1);

        driveGyro(-38,0.8, FACING_RIGHT);
        strafe(0.4, -4);
    }

}
