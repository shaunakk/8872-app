package org.firstinspires.ftc.robotcontroller.internal.program;
/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//BLUE TEAM
/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "ods sensor".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous//(name = "Sensor: MR ODS", group = "Sensor")
//@Disabled
public class autonomousV1 extends LinearOpMode {

    OpticalDistanceSensor odsSensor;  // Hardware Device Object
    DcMotor leftMotor;
    DcMotor rightMotor;
    Servo beaconPusher;
    LightSensor lightSensor;
    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 5;



    private ElapsedTime runtime = new ElapsedTime();
    static final double     WHITE_THRESHOLD = 0.4;  // spans between 0.1 - 0.5 from dark to light

    //0.32 = white, 0.22 = black

    static final double     APPROACH_SPEED  =0.2;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("color");

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);

        leftMotor = hardwareMap.dcMotor.get("motor_1");
        rightMotor = hardwareMap.dcMotor.get("motor_2");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        beaconPusher = hardwareMap.servo.get("beacon");
        // get a reference to our Light Sensor object.
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        lightSensor = hardwareMap.lightSensor.get("light_sensor");

        // wait for the start button to be pressed.

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.


        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        lightSensor.enableLed(true);

        beaconPusher.setPosition(0.2);


        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {

            telemetry.addData("Raw",    odsSensor.getRawLightDetected());
            telemetry.addData("Normal", odsSensor.getLightDetected());
            telemetry.addData("Light Level",  lightSensor.getLightDetected());

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        leftMotor.setPower(APPROACH_SPEED);
        rightMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < 0.31)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        encoderDrive(APPROACH_SPEED, 4, 4, 3.0);

        // Stop all motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setPower(0.1);


        //Turn until white
        while (opModeIsActive() && (lightSensor.getLightDetected() < 0.31)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);


        //line follow until reach beacon
        while (odsSensor.getRawLightDetected() < 0.2) {

            double scale;
            double powerIncrease = 0.075;
            double basePower = 0.075;

            double lightValue = lightSensor.getLightDetected();

            double lightError = (lightValue - 0.31);

            if (lightError == 0) { //between 0.26 and 0.28;    lightError <= 0.3 && lightError >= 0.28

                leftMotor.setPower(basePower);
                rightMotor.setPower(basePower);

            } else if (lightError < 0) {   //Too much black -> turn right

                scale = Math.abs(lightError) / basePower;

                leftMotor.setPower(basePower + powerIncrease * scale);
                rightMotor.setPower(basePower);

            } else if (lightError > 0) {   //Too much white -> turn left

                scale = Math.abs(lightError) / basePower;

                leftMotor.setPower(basePower);
                rightMotor.setPower(basePower + powerIncrease * scale);

            }

            telemetry.addData("Raw",    odsSensor.getRawLightDetected());
            telemetry.addData("Normal", odsSensor.getLightDetected());
            telemetry.addData("Light Level",  lightSensor.getLightDetected());

            telemetry.update();
            //idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

            sleep(10);

        }


        leftMotor.setPower(0);
        rightMotor.setPower(0);

        sleep(1000);


        //Back up four inches
        encoderDrive(APPROACH_SPEED, -4, -4, 3.0);

        //Turn servo

        beaconPusher.setPosition(0.78);

        sleep(1000);

        //Move forward and hit beacon
        encoderDrive(APPROACH_SPEED, 2.5, 2.5, 3.0);

        sleep(1000);

        // convert the RGB values to HSV values.
        //Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);

        int blueValue = sensorRGB.blue();
        int redValue = sensorRGB.red();
        int greenValue = sensorRGB.green();

        telemetry.addData("Red", redValue);
        telemetry.addData("Green", greenValue);
        telemetry.addData("Blue", blueValue);

        telemetry.update();
        sleep(1000);
        /*
        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", sensorRGB.alpha());
        telemetry.addData("Red  ", sensorRGB.red());
        telemetry.addData("Green", sensorRGB.green());
        telemetry.addData("Blue ", sensorRGB.blue());
        telemetry.addData("Hue", hsvValues[0]);
        */



        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });


        /*double checkTime = 5.0;

        runtime.reset();

        while(runtime.seconds() < checkTime) {

            telemetry.addData("Red", redValue);
            telemetry.addData("Green", blueValue);
            telemetry.addData("Blue", greenValue);

            telemetry.update();

        }*/


        encoderDrive(APPROACH_SPEED, -3, -3, 3.0);//back up four inches


        if (blueValue <= 500){ //if its red

            sleep(6000);

            //Move forward and hit beacon
            encoderDrive(APPROACH_SPEED, 3, 3, 3.0);

            sleep(1000);

            encoderDrive(APPROACH_SPEED, -3, -3, 3.0);//back up four inches
        }





    }

    public void encoderDrive(double speed, double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newRightTarget;
        int newLeftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rightMotor.setTargetPosition(newRightTarget);
            leftMotor.setTargetPosition(newLeftTarget);

            // Turn On RUN_TO_POSITION
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rightMotor.setPower(Math.abs(speed));
            leftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (rightMotor.isBusy() && leftMotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newRightTarget, newLeftTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        rightMotor.getCurrentPosition(),
                        leftMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}


//0.8 is a good distance from the wall