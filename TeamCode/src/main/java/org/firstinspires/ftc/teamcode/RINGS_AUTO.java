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


@Autonomous(name="RINGS_AUTO", group="Linear Opmode")
//@Disabled
public class RINGS_AUTO extends LinearOpMode {

    TechbotHardware         Terry   = new TechbotHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

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

        telemetry.addData("Status", "Resetting         // Send telemetry message to signify Terry waiting;\nEncoders");    //
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


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                Terry.leftDrive.getCurrentPosition(),
                Terry.rightDrive.getCurrentPosition(),
                Terry.leftBackDrive.getCurrentPosition(),
                Terry.rightBackDrive.getCurrentPosition());

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        encoderDrive(DRIVE_SPEED, 24, 24, 24, 24);

        encoderDrive(TURN_SPEED, 20, 20, -20, -20);
        //I have no idea how to turn so the chances of this being correct are slim

        encoderDrive(DRIVE_SPEED, 18, 18, 18, 18);

        Terry.wobbleClamp.setPosition(0.2);
        // Hand is closed on wobble

        encoderDrive(TURN_SPEED, -20, -20, 20, 20);
        //turn 90 degrees (measurements are incorrect)

        encoderDrive(DRIVE_SPEED, 25, 25, 25, 25);

        //Terry.servoHand.setPosition(0.2);
        //We don't know what thing to use shoot rings

        encoderDrive(SLIDER_SPEED, 12, 12, 12, 12);

        encoderDrive(TURN_SPEED, 20, 20, -20, -20);

        encoderDrive(DRIVE_SPEED, 48, 48, 48, 48);

        //Sense how many rings there are, we don't know how to do it yet
        //Line up to pick up rings

        Terry.tubeSpin.setPosition(0.2);
        //Pick up rings

        encoderDrive(TURN_SPEED, 20, 20, -20, -20);

        encoderDrive(DRIVE_SPEED, 48, 48, 48, 48);
        //back at launch line


        if () {
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





            //put in ring data

            //Terry.servoHand.setPosition(0.2);
            //We don't know what thing to use shoot rings

    }
}


