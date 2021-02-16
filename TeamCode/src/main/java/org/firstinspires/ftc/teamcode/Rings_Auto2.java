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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

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

    // Temporary variable to hold the label of the detected object
    String tempLabel = "no rings";

    private static final String VUFORIA_KEY =
            "ATmtSfv/////AAABmR8WElJ7jEXUlFRTowPWXCho/f+rytjnZJ9wA46OmtakoaZV1H9vnad9VQYLdEa1hpMdrslqRO2ZH6MuEvIb46HjosvUQcsMrMuQ8x3BdgmHIiJPEMjFkikP9gLt80K7hJRTT8EBZkXC7UsAB1JEC5v8p5gCwpYWkN6kua9ETIYTxrYmjnCpe+sKSv1LBCniFEvgah4ZASKiMaxEzwlJoQDlfIhVQ0YL4utkJ9A8+WbmHIybJO+ihRqc2eD6n1V86CLREtSZ1TXqicv0SMKnIHGxbCibFihtOl5orPO51HlNjZ2qxwE9EH9KOObpZq41fCiAd77VrqRCCmEr1McTrTfhk3TCTbBmc2SELnuysmZ5";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).


            // This is the zoom line
            tfod.setZoom(3, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        Terry.init(hardwareMap);
        // Start using camera to look for rings

        // Send telemetry message to signify Terry waiting;
        telemetry.addData("Status", "Resetting encoder");
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
        // Drive forward to align with rings
        encoderDrive(DRIVE_SPEED, 44, 44, 44, 44);

        telemetry.addData("Status", "Starting to detect");
        telemetry.update();

        // Object detection code
        /*if (opModeIsActive()) {
            for (int repeatDetection = 0; repeatDetection < 200; repeatDetection ++) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.addData(String.format("top (%d)", i), recognition.getTop());
                            tempLabel = recognition.getLabel();
                        }
                        telemetry.update();

                    }
                }
            }
        }

        //release tfod after getting label
        if (tfod != null) {
            tfod.shutdown();
        }

        // SPLIT CODE FOR TARGET ZONES

        // Target zone C if statement (4 rings)
        if (tempLabel == LABEL_FIRST_ELEMENT) {

            // Go forward to target zone C
            encoderDrive(DRIVE_SPEED, 100, 100, 100, 100);

            // Drop wobble and back up to launch line
            encoderDrive(DRIVE_SPEED, -50, -50, -50, -50);

            // Slide left to line up with first power shot
            encoderDrive(SLIDEL_SPEED, -34, 34, 34, -34);

            // Shoot one ring to first target
            // Slide to line up with the other power shot
            encoderDrive(SLIDEL_SPEED, -8, 8, 8, -8);

            // Shoot second power shot
            // Slide left to line up with third power shot
            encoderDrive(SLIDEL_SPEED,-13,13,13,-13);
            // Shoot power shot

            // Back up to get next to the wobble
            encoderDrive(DRIVE_SPEED, -74, -74, -74, -74);

            // Spin left to face wobble with front of robot
            encoderDrive(TURN_SPEED, 20, -20, 20, -20);

            // Go forward to get close to wobble
            encoderDrive(DRIVE_SPEED, 43, 43, 43, 43);

            // Turn right keeping wobble in front of robot
            encoderDrive(TURN_SPEED/2, 0, 34, -34, 34);

            // Go forward to put wobble in zone
            encoderDrive(DRIVE_SPEED, 122,122,122,122);

            // Back up to launch line
            encoderDrive(DRIVE_SPEED, -33, -33, -33, -33);


        } else if (tempLabel == LABEL_SECOND_ELEMENT) { */

            // Drift forward to line up with target zone B (1 ring) and turn
            encoderDrive(DRIVE_SPEED/2,55,93,93,93);

            // Drive forward to get wobble in target zone
            encoderDrive(DRIVE_SPEED/2,4,4,4,4);

            // Back away from wobble
            encoderDrive(DRIVE_SPEED/2, -10,-10,-10,-10);

            // Turn 90 degrees to face power shots
            encoderDrive(TURN_SPEED/2, 15,-15,15,-15);

            // Back up to get behind launch line
            encoderDrive(DRIVE_SPEED/2,-28,-28,-28,-28);

            // Slide to first power shot
            encoderDrive(SLIDEL_SPEED/2,-27,27,27,-27);

            // Shoot power shot
            // Slide left to line up with second power shot
            encoderDrive(SLIDEL_SPEED/2,-8,8,8,-8);

            // Shoot second power shot
            // Slide left to line up with third power shot
            encoderDrive(SLIDEL_SPEED/2,-13,13,13,-13);

            // Shoot third power shot
            // Back up to get next to the wobble
            encoderDrive(DRIVE_SPEED/2, -66, -66, -66, -66);

            // Spin left to face wobble with front of robot
            encoderDrive(TURN_SPEED/2, 20, -20, 20, -20);

            // Go forward to get close to wobble
            encoderDrive(DRIVE_SPEED/2, 43, 43, 43, 43);

            // Turn keeping wobble in front of robot
            encoderDrive(TURN_SPEED/2, 0,28,-28,28);

            // Drift forward to line up with target zone B (1 ring) and turn
            encoderDrive(DRIVE_SPEED/2,50,84,84,84);

            // Drive forward to get wobble in target zone
            encoderDrive(DRIVE_SPEED/2,10,10,10,10);

            // Back away from wobble
            encoderDrive(DRIVE_SPEED/2, 10,10,10,10);

            // Turn 90 degrees to face power shots
            encoderDrive(TURN_SPEED/2, 20,-20,20,-20);

            // Back up to get behind launch line
            encoderDrive(DRIVE_SPEED/2,-12,-12,-12,-12);


       /* } else {
            //drive to target zone A
            // telemetry.addData(String.format("label"), tempLabel);

            // telemetry.update();

            // Drive forward to target zone A (0 rings)
            encoderDrive(DRIVE_SPEED,30,30,30,30);

            // Slide left to line up with first power shot
            encoderDrive(SLIDEL_SPEED,-24,24,24,-24);

            // Shoot power shot
            // Slide left to line up with second power shot
            encoderDrive(SLIDEL_SPEED / 2,-8,8,8,-8);

            // Shoot power shot
            // Slide left to line up with third power shot
            encoderDrive(SLIDEL_SPEED / 2,-13,13,13,-13);
            // Shoot power shot

            // Back up to get next to the wobble
            encoderDrive(DRIVE_SPEED, -54, -54, -54, -54);

            // Spin left to face wobble with front of robot
            encoderDrive(TURN_SPEED, 20, -20, 20, -20);

            // Go forward to get close to wobble
            encoderDrive(DRIVE_SPEED, 36, 36, 36, 36);

            // Turn right keeping wobble in front of robot
            encoderDrive(TURN_SPEED / 2, 0, 34, -34, 34);

            // Go forward to put wobble in zone
            encoderDrive(DRIVE_SPEED, 47, 47, 47,47);

            // Back up away from wobble
            encoderDrive(DRIVE_SPEED, -8,-8,-8,-8);

            // Slide left away from target zone
            encoderDrive(SLIDEL_SPEED, -22,22,22,-22);

            // Drive forward onto the line
            encoderDrive(DRIVE_SPEED, 25,25,25,25);

        }

        telemetry.update(); */

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

                        telemetry.addData("target zone", tempLabel);

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
    // Defining function for vuforia
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "camera");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}

