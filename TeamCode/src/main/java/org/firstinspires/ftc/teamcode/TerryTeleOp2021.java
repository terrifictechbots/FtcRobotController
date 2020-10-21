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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//We're testing this comment

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TerryTeleOp2021", group="Linear Opmode")
//@Disabled
public class TerryTeleOp2021 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TechbotHardware Terry = new TechbotHardware();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        System.out.println("Is this thing working?!");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        Terry.init(hardwareMap);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double slidePower;
            double drivePower;
            double spinPowerCCW;
            double slideSpower;
            double spinPowerCW;
            double driveSPower;


            // Touch Sensors
            String touchSensorValue;
            if(Terry.touchSensor1.isPressed()) {
                touchSensorValue = "Pressed";
            } else {
                touchSensorValue = "Not Pressed";
            }
            telemetry.addData("touchSensor1", touchSensorValue);
            telemetry.update();

            if(Terry.touchSensor2.isPressed()) {
                touchSensorValue = "Pressed";
            } else {
                touchSensorValue = "Not Pressed";
            }
            telemetry.addData("touchSensor2", touchSensorValue);
            telemetry.update();

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.

            slidePower = gamepad1.right_stick_x;
            drivePower = gamepad1.right_stick_y;
            driveSPower = gamepad1.left_stick_y;
            spinPowerCCW = gamepad1.right_trigger;
            slideSpower = gamepad1.left_stick_x;
            spinPowerCW = -gamepad1.left_trigger;

            if (gamepad2.b) {
                Terry.wobbleLift.setPosition(0);
            }
            else {
                Terry.wobbleLift.setPosition(0.5);
            }

            if (gamepad2.right_bumper) {
                Terry.wobbleClamp.setPosition(0);
            }
            else {
                Terry.wobbleClamp.setPosition(0.5);
            }


            // Send calculated power to wheels

            if (gamepad1.right_stick_x > 0.2 || (gamepad1.right_stick_x < -0.2)) {
                Terry.slideL(slidePower / 2);
            } else if (gamepad1.right_stick_y > 0.2 || (gamepad1.right_stick_y < -0.2)) {
                Terry.drive(drivePower / 2);
            } else if (gamepad1.right_trigger > 0.2 || (gamepad1.right_trigger < -0.2)) {
                Terry.spin(spinPowerCCW / 2);
            } else if (gamepad1.left_stick_x > 0.2 || (gamepad1.left_stick_x < -0.2)) {
                Terry.slideS(slideSpower / 2);
            } else if (gamepad1.left_trigger > 0.2 || (gamepad1.left_trigger < -0.2)) {
                Terry.spin(spinPowerCW / 2);
            } else if (gamepad1.left_stick_y > 0.2 || (gamepad1.left_stick_y < -0.2)) {
                Terry.driveS(driveSPower);
            }

            else {
                Terry.drive(0);
                Terry.slideL(0);
                Terry.spin(0);
                Terry.spin(0);
                Terry.driveS(0);
                Terry.slideS(0);
            }


            Terry.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Terry.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Terry.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Terry.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
           // telemetry.addData("Motors", "left (%.2f), right (%.2f)", Terry.leftDrive, Terry.rightDrive, /*armPower, */Terry.leftBackDrive, Terry.rightBackDrive);
            /*telemetry.addData("Servo Position", "%5.2f", handPosition/*, wristPosition);*/
            //telemetry.update();
        }
    }
}