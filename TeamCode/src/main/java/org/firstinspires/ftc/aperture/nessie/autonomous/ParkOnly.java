package org.firstinspires.ftc.aperture.nessie.autonomous;

/**
 * BlueStonesAndWaffleAuto
 *
 * @author Aperture Science
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Park Only", group="PARK")
public class ParkOnly extends AutoCommon {

    private final double FACING_FORWARD = 0;

    @Override
    public void runOpMode() {

        super.runOpMode();

        // done with TF, shut down the LEDs
        robot.ledStrip.turnAllOff();
        turnOffVision();

        // pop out the slider once we start
        robot.servo4Bar.setPosition(0.5);
        driveGyro(9, 0.3, FACING_FORWARD);
    }
}
