package org.firstinspires.ftc.aperture.nessie.autonomous.blue;

/**
 * BlueWaffleStraightAuto
 *
 * @author Aperture Science
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.aperture.nessie.autonomous.AutoCommon;

@Autonomous(name="BLUE Waffle Straight Auto", group="BLUE")
public class BlueWaffleStraightAuto extends AutoCommon {

    @Override
    public void runOpMode() {
        super.runOpMode();

        // drive to the waffle
        driveGyro(26, 0.4, 0);
        strafe(.4,11);
        driveGyro(3, .2,0);

        // grab the waffle
        robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_DOWN);
        robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_DOWN);
        sleep(300);
        driveGyro(-17,.4,-10);
        driveGyro(-18,.4,0);

        // release the waffle
        robot.servoWaffleR.setPosition(robot.RIGHT_WAFFLE_UP);
        robot.servoWaffleL.setPosition(robot.LEFT_WAFFLE_UP);
        sleep(300);

        // drive to the middle line
        driveGyro(.5,.2,0);
        strafe(.4,-57);
        driveGyro(-3,.2,0);
    }

}
