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
public class BlueWallFollow extends LinearOpMode {
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
    Servo sideBeaconPusher;
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
        sideBeaconPusher = hardwareMap.servo.get("sidePush");
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

        sideBeaconPusher.setPosition(0);

        // Wait until we're told to go
        //waitForStart();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            //telemetry.addData("Raw", odsSensor.getRawLightDetected());
            //telemetry.addData("Normal", odsSensor.getLightDetected());
            telemetry.addData("Ultra", ultraSensor.getUltrasonicLevel());
            telemetry.addData("Side Pusher:", sideBeaconPusher.getPosition());
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

        encoderDrive(FASTER_SPEED, 24, 24, 6.0);

        sleep(100);

        //Fire 2 balls
        launcher.setPower(0.8);
        sleep(750);
        elevMotor.setPower(1.0);
        sleep(500);
        launcher.setPower(0.0);
        sleep(700);
        launcher.setPower(0.8);
        elevMotor.setPower(0.0);
        sleep(1250);
        launcher.setPower(0.0);


        encoderDrive(APPROACH_SPEED, -12, -12, 3.0);

        sleep(10);

        //beaconPusher.setPosition(0.8);

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






        //Turn 40 degrees
        double circumference = 13 * Math.PI;
        double turnDistance = (40/360) * circumference;

        encoderDrive(0.2, 6, -6, 6.0);

        sleep(50);

        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);



        if (enTelemetry == 1) {

            telemetry.addLine("Starting accurate turn");
            telemetry.update();


        }



        double basePower = 0.15;
        double powerIncrease = 0.1;
        double headingScale;

        double headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        while (headingAngle >= -53 || headingAngle <= -54) {


            //leftMotor.setPower(basePower + powerIncrease * headingScale);
            //rightMotor.setPower(-(basePower + powerIncrease * headingScale));
            if (headingAngle > -53) {
                rightMotor.setPower(-basePower);
                leftMotor.setPower(basePower);
            }

            if (headingAngle < -54) {
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

        encoderDrive(0.3, 49, 49, 6.0);


        sleep(100);

        if (testMode == 1) {
            while (!gamepad1.a) {
                sleep(10);
            }
        } else {

        }

        //encoderDrive(0.5, 16.5, -16.5, 8.0);


        leftMotor.setPower(APPROACH_SPEED);
        rightMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < 0.45)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }


        // Stop all motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        encoderDrive(APPROACH_SPEED,-6,6,3.0);

        headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        while (headingAngle <= -0.5 || headingAngle >= 0.5) {

            if (headingAngle < -0.5) {
                rightMotor.setPower(basePower);
                leftMotor.setPower(-basePower);
            }

            if (headingAngle > 0.5) {
                rightMotor.setPower(-basePower);
                leftMotor.setPower(basePower);
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

        double ultraLevel = ultraSensor.getUltrasonicLevel();
        telemetry.addData("Ultrasonic:", ultraLevel);
        telemetry.update();

        sleep(500);

        basePower = 0.15;

        headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < 0.40)) {

            if (ultraLevel > 16 && headingAngle < 0){

            }

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();

            headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }


        // Stop all motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        sleep(100);

        //angle following

        /*double anglePower = 0.1;
        powerIncrease = 0.05;

        leftMotor.setPower(anglePower);
        rightMotor.setPower(anglePower);

        sleep(1000);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < 0.45)) {

            if (headingAngle > -7.5) {
                rightMotor.setPower(anglePower);
                leftMotor.setPower(anglePower + powerIncrease);
            }

            if (headingAngle < -8.5) {
                rightMotor.setPower(anglePower + powerIncrease);
                leftMotor.setPower(anglePower);
            } else {
                rightMotor.setPower(anglePower);
                leftMotor.setPower(anglePower);
            }



            if (enTelemetry == 1) {
                //values.add(headingAngle);
                telemetry.update();
            }

            //sleep(50);

            //rightMotor.setPower(0);
            //leftMotor.setPower(0);

            sleep(10);

            headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }*/

        encoderDrive(APPROACH_SPEED, 3.5, 3.5, 3.0);

        headingAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        while (headingAngle <= -0.5 || headingAngle >= 0.5) {


            //leftMotor.setPower(basePower + powerIncrease * headingScale);
            //rightMotor.setPower(-(basePower + powerIncrease * headingScale));
            if (headingAngle < -0.5) {
                rightMotor.setPower(basePower);
                leftMotor.setPower(-basePower);
            }

            if (headingAngle > 0.5) {
                rightMotor.setPower(-basePower);
                leftMotor.setPower(basePower);
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

        // Stop all motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        sleep(500);
        // convert the RGB values to HSV values.
        //Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);

        int blueValue = sensorRGB.blue();
        int redValue = sensorRGB.red();
        int greenValue = sensorRGB.green();

        telemetry.addData("Red", redValue);
        telemetry.addData("Green", greenValue);
        telemetry.addData("Blue", blueValue);

        telemetry.update();

        sleep(500);

        if (blueValue <= 800) { //if its red

            sideBeaconPusher.setPosition(0.5);

        } else { //if blue

            sideBeaconPusher.setPosition(0.8);

        }

        sleep(500);

        sideBeaconPusher.setPosition(0);

        //Move back to close beacon
        leftMotor.setPower(-APPROACH_SPEED);
        rightMotor.setPower(-APPROACH_SPEED);

        sleep(1000);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < 0.40)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }


        // Stop all motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        encoderDrive(APPROACH_SPEED, 3.5, 3.5, 3.0);

        //hit second beacon

        blueValue = sensorRGB.blue();
        redValue = sensorRGB.red();
        greenValue = sensorRGB.green();

        telemetry.addData("Red", redValue);
        telemetry.addData("Green", greenValue);
        telemetry.addData("Blue", blueValue);

        telemetry.update();

        sleep(500);

        if (blueValue <= 800) { //if its red

            sideBeaconPusher.setPosition(0.5);

        } else { //if blue

            sideBeaconPusher.setPosition(0.8);

        }

        sleep(500);

    }

    // Loop and update the dashboard
    public void encoderDrive(double speed, double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newRightTarget;
        int newLeftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
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
                    (rightMotor.isBusy() && leftMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newRightTarget, newLeftTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
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




