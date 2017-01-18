package org.firstinspires.ftc.robotcontroller.internal.program;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by administrator on 9/26/16.
 */

public class Test extends OpMode{

    DcMotor motor_1;
    DcMotor motor_2;

    @Override
    public void init() {
        //get reference to the motors from the hardware map
        motor_1 = hardwareMap.dcMotor.get("motor_1");
        motor_2 = hardwareMap.dcMotor.get("motor_2");

        //reverse the left motor
        motor_2.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        //get the values from the gamepads
        //note: pushing the stick all the way up returns -1, so we need to revese the values
        float xValue = -gamepad1.left_stick_y;
        float yValue = -gamepad1.right_stick_y;

        //calculate the power needed for each motor
        float leftPower = yValue + xValue;
        float rightPower = yValue + xValue;

        //
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        //set the power of the motors with the gamepad vlaues
        motor_1.setPower(leftPower);
        motor_2.setPower(rightPower);
    }


}
