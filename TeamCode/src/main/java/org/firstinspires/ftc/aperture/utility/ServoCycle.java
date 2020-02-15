/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.aperture.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.aperture.nessie.RobotHardware;


@TeleOp(name="Servo Cycle", group="Utility")
public class ServoCycle extends LinearOpMode {

    private RobotHardware robot;
    private boolean prevGamepad = false;
    private boolean prevBumper = false;
    private int which = 0;

    @Override
    public void runOpMode() {

        robot = new RobotHardware(hardwareMap, false);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Servo[] servos    = { robot.servoCapGripper, robot.servoCap, robot.servo4Bar, robot.servoGrip, robot.servoWaffleR, robot.servoWaffleL};
        String[] names     = { "servoCapGripper", "servoCap", "servo4Bar", "servoGrip", "servoWaffleR", "servoWaffleL"};
        Double[] positions = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };



        waitForStart();

        while (opModeIsActive()) {

            // servo mod
            if (gamepad1.a && !prevGamepad) positions[which] += .05;
            if (gamepad1.b && !prevGamepad) positions[which] -= .05;
            if (gamepad1.x && !prevGamepad) positions[which] += .01;
            if (gamepad1.y && !prevGamepad) positions[which] -= .01;
            prevGamepad = gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y;

            if (gamepad1.left_bumper && !prevBumper) which--;
            if (which < 0) which = servos.length - 1;
            if (gamepad1.right_bumper && !prevBumper) which++;
            if (which > servos.length - 1) which = 0;
            prevBumper = gamepad1.left_bumper || gamepad1.right_bumper;

            // modify range
            positions[which] = Range.clip(positions[which], 0, 1);

            // set position
            servos[which].setPosition(positions[which]);

            // add telemetry
            telemetry.addLine(names[which]);
            telemetry.addData("Position", positions[which]);
            telemetry.update();
        }
    }
}