package org.firstinspires.ftc.robotcontroller.internal.program;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by administrator on 9/28/16.
 */

public class ballarm extends LinearOpMode{
    DcMotor armMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        armMotor = hardwareMap.dcMotor.get("armmotor");


        armMotor.setPower(1);

        sleep(200);

        armMotor.setPower(0.0);

    }
}