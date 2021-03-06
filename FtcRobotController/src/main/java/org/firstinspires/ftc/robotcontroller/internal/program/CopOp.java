/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.internal.program;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="CopOp", group="Pushbot")
//@Disabled
public class CopOp extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor rightMotor;
    DcMotor leftMotor;

    DcMotor elevMotor;
    DcMotor launcher;

    DcMotor capMotor;

    Servo beaconPusher;
    Servo capServo;
    Servo ballServo;

    private ElapsedTime runtime = new ElapsedTime();

    boolean sweeperValue = true;
    boolean capMode = true;
    boolean capValue = true;

    static final double APPROACH_SPEED = 0.2;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double launch;
        double max;
        double elevControl;
        double launchControl;
        double capControl;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        leftMotor = hardwareMap.dcMotor.get("motor_1");
        rightMotor = hardwareMap.dcMotor.get("motor_2");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        elevMotor = hardwareMap.dcMotor.get("elev");
        launcher = hardwareMap.dcMotor.get("launch");
        launcher.setDirection(DcMotor.Direction.REVERSE);
        capMotor = hardwareMap.dcMotor.get("cap");
        elevMotor.setDirection(DcMotor.Direction.REVERSE);

        beaconPusher = hardwareMap.servo.get("beacon");
        capServo = hardwareMap.servo.get("capLock");
        ballServo = hardwareMap.servo.get("ballServo");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();


        beaconPusher.setPosition(1.0);

        ballServo.setPosition(0);

        capServo.setPosition(0.5);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            launch =  -gamepad2.left_stick_y;
            capControl = -gamepad2.right_stick_y;

            leftMotor.setPower(left);
            rightMotor.setPower(right);
            launcher.setPower(launch);
            capMotor.setPower(capControl);

            // Use gamepad1 to control beaconpusher(position values are not determined yet!!!!!)
            if (gamepad1.b) {
                beaconPusher.setPosition(-0.1);
            }

            if (gamepad1.a) {
                beaconPusher.setPosition(1.1);
            }

            //Press a on secondary controller to drop cap lifter
            if (gamepad2.x) {

                if (capValue == true){
                    capServo.setPosition(0);
                    capValue = false;
                } else if (capValue == false) {
                    capServo.setPosition(0.5);
                    capValue = true;
                }
            }

            //Use gamepad2 to control elevator/sweeper
            if (gamepad2.a){

                if (sweeperValue == true){
                    elevMotor.setPower(1.0);
                    sweeperValue = false;
                } else if (sweeperValue == false) {
                    elevMotor.setPower(0.0);
                    sweeperValue = true;
                }
            }

//            if (gamepad1.y){
//
//                capMode = true;
//
//
//
//                sleep(500);
//
//                while (capMode = true) {
//                    right = (gamepad1.left_stick_y) / 2;
//                    left = (gamepad1.right_stick_y) / 2;
//                    capMotor.setPower(capControl);
//
//                    if (gamepad1.y) {
//                        capMode = false;
//
//                        sleep(500);
//                    }
//
//                    sleep(40);
//                }
//
//
//
//            }








            // Use gamepad2 to control launcher
            if (gamepad2.b) {

                ballServo.setPosition(1.0);
                sleep(1000);
                ballServo.setPosition(0);
            }



            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }

    public void encoderDrive(double speed, double launchers,
                             double timeoutS) throws InterruptedException {

        int newLauncherTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //(int) (launchers * COUNTS_PER_INCH)

            // Determine new target position, and pass to motor controller
            newLauncherTarget = launcher.getCurrentPosition() + 4480;
            launcher.setTargetPosition(newLauncherTarget);

            // Turn On RUN_TO_POSITION
            launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            launcher.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (launcher.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d", newLauncherTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d", launcher.getCurrentPosition());
//                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            launcher.setPower(0);

            // Turn off RUN_TO_POSITION
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
