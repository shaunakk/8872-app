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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

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

@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
//@Disabled
public class TeleOpV1 extends LinearOpMode {

    /* Declare OpMode members. */
    UltrasonicSensor ultraSensor;
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor launcher;
    DcMotor elevMotor;
    DcMotor sweeper;
    Servo beaconPusher;
    DeviceInterfaceModule cdim;
    // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    //double          clawOffset      = 0;                       // Servo mid position
    //final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double max;
        double launchControl;
        double elevControl;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        leftMotor = hardwareMap.dcMotor.get("motor_1");
        rightMotor = hardwareMap.dcMotor.get("motor_2");
        beaconPusher = hardwareMap.servo.get("beacon");
        launcher =  hardwareMap.dcMotor.get("launch");
        elevMotor = hardwareMap.dcMotor.get("elev");
        sweeper = hardwareMap.dcMotor.get("sweep");
        launcher.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        ultraSensor = hardwareMap.ultrasonicSensor.get("ultra");


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.


            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

//            if (ultraSensor.getUltrasonicLevel() < 16) {
//                left = -(gamepad1.left_stick_y)/4;
//                right = -(gamepad1.right_stick_y)/4 ;
//            }

            leftMotor.setPower(left);
            rightMotor.setPower(right);

            leftMotor.setPower(left);
            rightMotor.setPower(right);



            // Use gamepad left & right Bumpers to open and close the claw
            // Use gamepad1 to control beaconpusher(position values are not determined yet!!!!!)
            if (gamepad1.b) {
                beaconPusher.setPosition(0.65);
            }
            else if (gamepad1.a) {
                beaconPusher.setPosition(1.0);
            }



            launchControl = -gamepad2.right_stick_y;

            if (launchControl >= 0.2) {

                launcher.setPower(launchControl-0.2);
            }
            else {
                launcher.setPower(0);
            }

            elevControl = -gamepad2.left_stick_y;
            elevMotor.setPower(elevControl);

            if (gamepad2.a) {
                sweeper.setPower(1.0);
            }
            else if (gamepad2.b) {
                sweeper.setPower(0.0);
            }
            // Send telemetry message to signify robot running;
            //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("left motor power", "%.2f", leftMotor.getPower());
            telemetry.addData("right motor power", "%.2f", rightMotor.getPower());
            telemetry.addData("ultra", ultraSensor.getUltrasonicLevel());
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
