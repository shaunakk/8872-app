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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;

import static com.qualcomm.hardware.adafruit.BNO055IMU.GyroBandwidth.HZ523;


/**
 * {@link //SensorAdafruitIMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous//(name = "Sensor: Adafruit IMU", group = "Sensor")
//@Disabled                            // Uncomment this to add to the opmode list
public class NewDoubleBeaconProgramThatIsForWinningRegionalsRed extends LinearOpMode {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    //OpticalDistanceSensor odsSensor;  // Hardware Device Object
    UltrasonicSensor ultraSensor;
    DcMotor launcher;
    DcMotor elevMotor;
    Servo beaconPusher;
    Servo ballServo;
    Servo capServo;
    LightSensor lightSensor;
    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;
    DcMotor leftMotor;
    DcMotor rightMotor;

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 5;


    private ElapsedTime runtime = new ElapsedTime();
    static final double WHITE_THRESHOLD = 0.4;  // spans between 0.1 - 0.5 from dark to light

    //0.32 = white, 0.22 = black
    static final double FASTER_SPEED = 0.3;
    static final double APPROACH_SPEED = 0.2;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);


    int testMode = 0;
    int enTelemetry = 1;
    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {
// hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

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
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.gyroBandwidth = HZ523;

        leftMotor = hardwareMap.dcMotor.get("motor_1");
        rightMotor = hardwareMap.dcMotor.get("motor_2");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        launcher = hardwareMap.dcMotor.get("launch");
        launcher.setDirection(DcMotor.Direction.REVERSE);
        elevMotor = hardwareMap.dcMotor.get("elev");


        beaconPusher = hardwareMap.servo.get("beacon");
        ballServo = hardwareMap.servo.get("ballServo");
        capServo = hardwareMap.servo.get("capLock");
        // get a reference to our Light Sensor object.
        //odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        ultraSensor = hardwareMap.ultrasonicSensor.get("ultra");
        lightSensor = hardwareMap.lightSensor.get("light_sensor");
        //float heading = imu.getAngularOrientation().firstAngle;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // wait for the start button to be pressed.

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.


        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        lightSensor.enableLed(true);

        beaconPusher.setPosition(1.0);

        ballServo.setPosition(0);

        capServo.setPosition(0.5);

        // Wait until we're told to go
        //waitForStart();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            telemetry.addData("Ultra", ultraSensor.getUltrasonicLevel());
            telemetry.addData("storage", ballServo.getPosition());
            telemetry.addData("beacon" , beaconPusher.getPosition());
            telemetry.addData("Normal", ultraSensor.getUltrasonicLevel());
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        //beaconPusher.setPosition(0.2);

        sleep(100);

        encoderDrive(0.4, 13, 13, 7.0);

        sleep(100);

        if (testMode == 1) {
            while (!gamepad1.a) {
                sleep(10);
            }
        } else {

        }


//        encoderDrive(0.4, 0, 0, 12, 5.0);
        //launch 2 balls:
        launcher.setPower(0.8);
        sleep(1300);
        launcher.setPower(0);
        ballServo.setPosition(1.0);
        sleep(750);
        ballServo.setPosition(0);
        sleep(300);
        launcher.setPower(0.8);
        sleep(1300);
        launcher.setPower(0);

        sleep(100);


        //Back up
//        encoderDrive(0.3, -6, -6, 6.0);

        if (testMode == 1) {
            while (!gamepad1.a) {
                sleep(10);
            }
        } else {

        }

        if (enTelemetry == 1) {
            telemetry.addLine("Starting telemetry");
            telemetry.update();



            composeTelemetry();

            telemetry.addLine("Telemetry finished");
            telemetry.update();



            telemetry.addLine("Starting fast turn");
            telemetry.update();


        }


        // Set up our telemetry dashboard

        //Turn towards first white line
        encoderDrive(0.3, -5.6, 5.6, 6.0);


        sleep(50);

        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);



        if (enTelemetry == 1) {

            telemetry.addLine("Starting accurate turn");
            telemetry.update();


        }


        //Gyro Turn to first white line
        double basePower = 0.4;
        double powerIncrease = 0.1;
        double headingScale;

        double headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        // orginal was -41 to -43

        while (headingAngle <= 56 || headingAngle >= 58) {


            //leftMotor.setPower(basePower + powerIncrease * headingScale);
            //rightMotor.setPower(-(basePower + powerIncrease * headingScale));
            if (headingAngle <= 56) {
                rightMotor.setPower(basePower);
                leftMotor.setPower(-basePower);
            }

            if (headingAngle >= 58) {
                rightMotor.setPower(-basePower );
                leftMotor.setPower(basePower );
            }

            if (enTelemetry == 1) {
                //values.add(headingAngle);
                telemetry.update();
            }

            sleep(50);

            rightMotor.setPower(0);
            leftMotor.setPower(0);

            sleep(10);

            headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));



        }

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        sleep(100);

        if (enTelemetry == 1) {
            telemetry.addData("Left Power", leftMotor.getPower());
            telemetry.addData("Right Power", rightMotor.getPower());
            telemetry.update();


        }
        if (testMode == 1) {
            while (!gamepad1.a) {
                sleep(10);
            }
        } else {

        }


        //Drive forward
        encoderDrive(0.4, 20, 20, 9.0);


        sleep(100);

        if (testMode == 1) {
            while (!gamepad1.a) {
                sleep(10);
            }
        } else {

        }

        //encoderDrive(0.5, 16.5, -16.5, 8.0);


        leftMotor.setPower(0.1);
        rightMotor.setPower(0.1);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < 0.4)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }


        // Stop all motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        //Move forward past white line
        encoderDrive(APPROACH_SPEED, 3, 3, 3.0);

        //encoderDrive(APPROACH_SPEED, (3+4.5)/2, (3-4.5)/2, 3.0);

        //Turn to face beacon
        encoderDrive(APPROACH_SPEED, -2.5, 2.5, 3.0);

        headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        while (headingAngle >= 91 || headingAngle <= 89) {



            if (headingAngle >= 91) {
                rightMotor.setPower(-basePower);
                leftMotor.setPower(basePower);
            }

            if (headingAngle <= 89) {
                rightMotor.setPower(basePower);
                leftMotor.setPower(-basePower);
            }

            if (enTelemetry == 1) {
                //values.add(headingAngle);
                telemetry.update();
            }

            sleep(50);

            rightMotor.setPower(0);
            leftMotor.setPower(0);

            sleep(10);

            headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));



        }


        double ultraLevel = ultraSensor.getUltrasonicLevel();
        telemetry.addData("Ultrasonic:", ultraLevel);
        telemetry.update();

        sleep(100);

        //Drive forward until ultradistance
        runtime.reset();
        leftMotor.setPower(0.1);
        rightMotor.setPower(0.1);


        while (ultraSensor.getUltrasonicLevel() > 13.0 && runtime.seconds() < 3) {
            telemetry.addData("ultra", ultraSensor.getUltrasonicLevel());

            telemetry.update();

            sleep(10);
        }

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        runtime.reset();

        // convert the RGB values to HSV values.
        //Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);



        //Detect color
        int blueValue = sensorRGB.blue();
        int redValue = sensorRGB.red();
        int greenValue = sensorRGB.green();

        telemetry.addData("Red", redValue);
        telemetry.addData("Green", greenValue);
        telemetry.addData("Blue", blueValue);

        telemetry.update();



//        relativeLayout.post(new Runnable() {
//            public void run() {
//                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
//            }
//        });

        //Push correct beacon
        if (redValue > blueValue) { //if its red
            //Move forward and hit beacon
            beaconPusher.setPosition(0);
            sleep(300);

//            encoderDrive(APPROACH_SPEED, 2, 2, 3.0);
            leftMotor.setPower(APPROACH_SPEED);
            rightMotor.setPower(APPROACH_SPEED);

            runtime.reset();

            while (ultraSensor.getUltrasonicLevel() > 5.0 && runtime.seconds() < 2) {
                telemetry.addData("ultra", ultraSensor.getUltrasonicLevel());

                telemetry.update();

                sleep(10);
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            //sleep(100); ADD THIS IF IT DOESNT HIT THE BEACON

        } else if (redValue < blueValue) { //if its blue

            //Move forward and hit beacon
//            encoderDrive(APPROACH_SPEED, 2, 2, 3.0);
            sleep(300);

            leftMotor.setPower(APPROACH_SPEED);
            rightMotor.setPower(APPROACH_SPEED);

            runtime.reset();

            while (ultraSensor.getUltrasonicLevel() > 5.0 && runtime.seconds() < 2) {
                telemetry.addData("ultra", ultraSensor.getUltrasonicLevel());

                telemetry.update();

                sleep(10);
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            //sleep(100); ADD THIS IF IT DOESNT HIT THE BEACON

        }

        encoderDrive(0.4 , -40, -50, 10);

//        encoderDrive(APPROACH_SPEED, -4.5, -4.5, 6.0);//back up 5 inches
//
//        encoderDrive(FASTER_SPEED, 8.5, -8.5, 6.0);
//
//        //Turn to face next white line
//        headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
//
//        basePower = 0.4;
//
//        while (headingAngle >= 5 || headingAngle <= 3) {
//
//
//
//            if (headingAngle >= 5) {
//                rightMotor.setPower(-basePower);
//                leftMotor.setPower(basePower);
//            }
//
//            if (headingAngle <= 3) {
//                rightMotor.setPower(basePower);
//                leftMotor.setPower(-basePower);
//            }
//
//            if (enTelemetry == 1) {
//                //values.add(headingAngle);
//                telemetry.update();
//            }
//
//            sleep(50);
//
//            rightMotor.setPower(0);
//            leftMotor.setPower(0);
//
//            sleep(10);
//
//            headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
//
//
//
//        }
//
//
//        //Drive to second white line
//        beaconPusher.setPosition(1.0);
//
//        encoderDrive(0.4, 23, 23, 4.0);
//
//        leftMotor.setPower(0.1);
//        rightMotor.setPower(0.1);
//
//        runtime.reset();
//
//        //Drive with line detection
//        while (opModeIsActive() && (lightSensor.getLightDetected() < 0.4)) {
//
//            // Display the light level while we are looking for the line
//            telemetry.addData("Light Level", lightSensor.getLightDetected());
//            telemetry.update();
//            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
//        }
//
//        leftMotor.setPower(0.0);
//        rightMotor.setPower(0.0);
//
//        encoderDrive(APPROACH_SPEED, 3, 3, 4.0);
//
//        //Turn to beacon
//        encoderDrive(APPROACH_SPEED, -8, 8, 3.0);
//
//        headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
//
//        while (headingAngle >= 91 || headingAngle <= 89) {
//
//            if (headingAngle >= 91) {
//                rightMotor.setPower(-basePower);
//                leftMotor.setPower(basePower);
//            }
//
//            if (headingAngle <=  89) {
//                rightMotor.setPower(basePower);
//                leftMotor.setPower(-basePower);
//            }
//
//            if (enTelemetry == 1) {
//                //values.add(headingAngle);
//                telemetry.update();
//            }
//
//            sleep(50);
//
//            rightMotor.setPower(0);
//            leftMotor.setPower(0);
//
//            sleep(10);
//
//            headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
//
//
//
//        }
//
//
//        //Drive until ultradistance
//        leftMotor.setPower(0.1);
//        rightMotor.setPower(0.1);
//
//        while (ultraSensor.getUltrasonicLevel() > 12.0 && runtime.seconds() < 3.0) {
//            telemetry.addData("ultra", ultraSensor.getUltrasonicLevel());
//
//            telemetry.update();
//
//            sleep(10);
//        }
//
//        leftMotor.setPower(0.0);
//        rightMotor.setPower(0.0);
//
//        runtime.reset();
//
//
//        blueValue = sensorRGB.blue();
//        redValue = sensorRGB.red();
//        greenValue = sensorRGB.green();
//
//        telemetry.addData("Red", redValue);
//        telemetry.addData("Green", greenValue);
//        telemetry.addData("Blue", blueValue);
//
//        telemetry.update();
//
////        relativeLayout.post(new Runnable() {
////            public void run() {
////                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
////            }
////        });
//
////Push correct beacon
//        if (redValue > blueValue) { //if its red
//            //Move forward and hit beacon
//            beaconPusher.setPosition(0);
//            sleep(300);
//
////            encoderDrive(APPROACH_SPEED, 2, 2, 3.0);
//            leftMotor.setPower(APPROACH_SPEED);
//            rightMotor.setPower(APPROACH_SPEED);
//
//            runtime.reset();
//
//            while (ultraSensor.getUltrasonicLevel() > 5.0 && runtime.seconds() < 2) {
//                telemetry.addData("ultra", ultraSensor.getUltrasonicLevel());
//
//                telemetry.update();
//
//                sleep(10);
//            }
//            leftMotor.setPower(0);
//            rightMotor.setPower(0);
//
//            //sleep(100); ADD THIS IF IT DOESNT HIT THE BEACON
//
//        } else if (redValue < blueValue) { //if its blue
//
//            //Move forward and hit beacon
////            encoderDrive(APPROACH_SPEED, 2, 2, 3.0);
//
//            leftMotor.setPower(APPROACH_SPEED);
//            rightMotor.setPower(APPROACH_SPEED);
//
//            runtime.reset();
//
//            while (ultraSensor.getUltrasonicLevel() > 5.0 && runtime.seconds() < 2) {
//                telemetry.addData("ultra", ultraSensor.getUltrasonicLevel());
//
//                telemetry.update();
//
//                sleep(10);
//            }
//            leftMotor.setPower(0);
//            rightMotor.setPower(0);
//
//            //sleep(100); ADD THIS IF IT DOESNT HIT THE BEACON
//
//        }
//
//        //Back up
//        encoderDrive(0.4, -13, -13, 4.0);
//        sleep(100);
//
//        //Turn for cap ball
//        encoderDrive(0.4, 5, -5, 4.0);
//        sleep(100);
//
//        //back up to hit cap ball
//        encoderDrive(0.5, -50, -50, 7.0);

    }

    // Loop and update the dashboard
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }




    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}




