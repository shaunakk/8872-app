package org.firstinspires.ftc.robotcontroller.internal.program;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by administrator on 9/26/16.
 */

public class Test2 extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public  void init() {

        leftMotor = hardwareMap.dcMotor.get("motor_1");
        rightMotor = hardwareMap.dcMotor.get("motor_2");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        float xValue = -gamepad1.left_stick_x;
        float yValue = -gamepad1.left_stick_y;

        float leftPower = yValue + xValue;
        float rightPower = yValue - xValue;

        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

    }

}
