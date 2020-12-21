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
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S ENT RIGHTS ARE GRANTED BY THIS
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.Terrycontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Rings_Auto2", group="Linear Opmode")
//@Disabled
public class Rings_Auto2 extends LinearOpMode {

    /* Declare OpMode members. */
    TechbotHardware        Terry   = new TechbotHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder - 1440
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double SLIDEL_SPEED = 0.6;
    static final double SLIDER_SPEED = 0.6;
    static final double STOP = 0;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        Terry.init(hardwareMap);
        // Start using camera to look for rings

        // Send telemetry message to signify Terry waiting;
        telemetry.addData("Status", "Resetting encoder");    //
        telemetry.update();

        Terry.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Terry.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Terry.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Terry.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Terry.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Terry.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Terry.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Terry.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Turn On RUN_TO_POSITION
        Terry.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Terry.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Terry.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Terry.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        //Rings_Auto starts here

        // Start with back against the wall with the camera forward
        // Holding wobble and 3 rings
        // Drive forward to the target zone
        encoderDrive(DRIVE_SPEED/2, 115, 115, 115, 115);
        // Drop wobble
        encoderDrive(DRIVE_SPEED/2, -40,-40,-40,-40);

        // Slide left to line up with first power shot
        encoderDrive(SLIDEL_SPEED/2, -26,26,26,-26);

        // Slide to the front of the target
        encoderDrive(SLIDEL_SPEED/2, -8, 8, 8, -8);

        // Shoot one ring to first target
        // Slide to line up with the other power shot
        encoderDrive(SLIDEL_SPEED/2, -8, 8, 8, -8);

        // Shoot second power shot
        // Slide to third power shot
        encoderDrive(SLIDEL_SPEED/2, -8, 8, 8, -8);

        // Shoot third power shot

        // Back up to get next to the wobble
        encoderDrive(DRIVE_SPEED/2, -60,-60,-60,-60);

        // Spin left to face wobble with wobble arm
        encoderDrive(TURN_SPEED/2, -20,20,-20,20);

        // Back up to get close to wobble
        encoderDrive(DRIVE_SPEED/2,-20,-20,-20,-20);

        // Pick up wobble
        // Turn right to get ring pick up near rings
        encoderDrive(TURN_SPEED/2, 20,-20,20,-20);

        // Go forward to get rings close to ring pick up
        encoderDrive(DRIVE_SPEED/2,20,20,20,20);

        // Lower ring pick up arm
        // Pick up rings
        // Raise pick up arm
        // Go forward to line up with target zone C
        encoderDrive(DRIVE_SPEED/2, 80,80,80,80);

        // Turn left to face target zone C
        encoderDrive(TURN_SPEED/2, -20,20,-20,20);

        // Drop wobble
        // Slide left to launch line
        encoderDrive(SLIDEL_SPEED/2, -40,40,40,-40);

        /* if () {
            encoderDrive(TURN_SPEED, 20, 20, -20, -20);
            encoderDrive(DRIVE_SPEED, 24, 24, 24, 24);
            Terry.wobbleClamp.setPosition(0.2);
            encoderDrive(TURN_SPEED, -20, -20, 20, 20);
            encoderDrive(DRIVE_SPEED, -2, -2, -2, -2);
        } else if () {
            encoderDrive(DRIVE_SPEED, 24, 24, 24, 24);
            encoderDrive(TURN_SPEED, 20, 20, -20, -20);
            Terry.wobbleClamp.setPosition(0.2);
            encoderDrive(TURN_SPEED, -20, -20, 20, 20);
            encoderDrive(DRIVE_SPEED, -26, -26, -26, -26);
            encoderDrive(SLIDEL_SPEED, 12, 12, 12, 12);
        } else if () {
            encoderDrive(DRIVE_SPEED, 48, 48, 48, 48);
            encoderDrive(TURN_SPEED, 20, 20, -20, -20);
            encoderDrive(DRIVE_SPEED, 24, 24, 24, 24);
            Terry.wobbleClamp.setPosition(0.2);
            encoderDrive(SLIDEL_SPEED, 50, 50, 50, 50);
            encoderDrive(TURN_SPEED, -20, -20, 20, 20);
        }
         */
        //Rings_Auto ends here
    }

    /*
     *  Method to preform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */




            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the Terry will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the Terry continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            //while (Terry.rightDrive.getCurrentPosition() < newRightTarget && opModeIsActive()) {

    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double leftBackInches, double rightBackInches) {
                int newLeftTarget;
                int newRightTarget;
                int newLeftBackTarget;
                int newRightBackTarget;

                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    newLeftTarget = Terry.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    newRightTarget = Terry.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                    newLeftBackTarget = Terry.leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
                    newRightBackTarget = Terry.rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);


                    Terry.leftDrive.setTargetPosition(newLeftTarget);
                    Terry.rightDrive.setTargetPosition(newRightTarget);
                    Terry.leftBackDrive.setTargetPosition(newLeftBackTarget);
                    Terry.rightBackDrive.setTargetPosition(newRightBackTarget);

                    // reset the timeout time and start motion.
                    runtime.reset();
                    Terry.leftDrive.setPower(Math.abs(speed));
                    Terry.rightDrive.setPower(Math.abs(speed));
                    Terry.leftBackDrive.setPower(Math.abs(speed));
                    Terry.rightBackDrive.setPower(Math.abs(speed));

                    while (opModeIsActive() && (Terry.rightDrive.isBusy() || Terry.leftBackDrive.isBusy() || Terry.leftDrive.isBusy() || Terry.leftBackDrive.isBusy()))
                    {

                        // Display it for the driver.
                        telemetry.addData("TargetPos", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
                        telemetry.addData("CurrentPos", "Running at %7d :%7d :%7d :%7d",
                                Terry.leftDrive.getCurrentPosition(),
                                Terry.rightDrive.getCurrentPosition(),
                                Terry.leftBackDrive.getCurrentPosition(),
                                Terry.rightBackDrive.getCurrentPosition());

                        telemetry.update();
                    }

                    Terry.leftDrive.setPower(0);
                    Terry.rightDrive.setPower(0);
                    Terry.leftBackDrive.setPower(0);
                    Terry.rightBackDrive.setPower(0);

                    Terry.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Terry.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Terry.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Terry.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    Terry.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Terry.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Terry.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Terry.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    // Turn On RUN_TO_POSITION
                    Terry.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Terry.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Terry.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Terry.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    }
}

