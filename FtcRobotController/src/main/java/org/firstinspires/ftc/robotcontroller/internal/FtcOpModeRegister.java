/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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

package org.firstinspires.ftc.robotcontroller.internal;

import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeRegistrar;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptNullOp;
//import org.firstinspires.ftc.robotcontroller.internal.testcode.ArmLauncherTest;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorAdafruitRGB;
import org.firstinspires.ftc.robotcontroller.internal.program.BallShootingBlue;
import org.firstinspires.ftc.robotcontroller.internal.program.BallShootingRed;
import org.firstinspires.ftc.robotcontroller.internal.program.BlueCloseBeaconV1;
import org.firstinspires.ftc.robotcontroller.internal.program.BlueBeacon2;
import org.firstinspires.ftc.robotcontroller.internal.program.BlueFarBeaconV1;
import org.firstinspires.ftc.robotcontroller.internal.program.BlueSideBeacon;;
import org.firstinspires.ftc.robotcontroller.internal.program.BlueWallFollow;
import org.firstinspires.ftc.robotcontroller.internal.program.CopOp;
import org.firstinspires.ftc.robotcontroller.internal.program.DrivingChassis;
import org.firstinspires.ftc.robotcontroller.internal.program.GyroCrap;
import org.firstinspires.ftc.robotcontroller.internal.program.NewDoubleBeaconProgramThatIsForWinningRegionalsBlue;
import org.firstinspires.ftc.robotcontroller.internal.program.NewDoubleBeaconProgramThatIsForWinningRegionalsRed;
import org.firstinspires.ftc.robotcontroller.internal.program.RedCloseBeaconV1;
import org.firstinspires.ftc.robotcontroller.internal.program.RedBeacon2;
import org.firstinspires.ftc.robotcontroller.internal.program.RedFarBeaconV1;
import org.firstinspires.ftc.robotcontroller.internal.program.TeleOpV3;
import org.firstinspires.ftc.robotcontroller.internal.program.driveOnly;
import org.firstinspires.ftc.robotcontroller.internal.program.gyroTestV1;
import org.firstinspires.ftc.robotcontroller.internal.program.gyroTestV2;
import org.firstinspires.ftc.robotcontroller.internal.program.omnidrive;
import org.firstinspires.ftc.robotcontroller.internal.program.rgbsensor;
import org.firstinspires.ftc.robotcontroller.internal.program.DriveBotTank;
import org.firstinspires.ftc.robotcontroller.internal.program.Test;
import org.firstinspires.ftc.robotcontroller.internal.program.Test2;
import org.firstinspires.ftc.robotcontroller.internal.program.autonomousV1;
import org.firstinspires.ftc.robotcontroller.internal.program.autonomousV2;
import org.firstinspires.ftc.robotcontroller.internal.program.autonomousV3;
import org.firstinspires.ftc.robotcontroller.internal.program.ballarm;
import org.firstinspires.ftc.robotcontroller.internal.program.beaconpusherV1;
import org.firstinspires.ftc.robotcontroller.internal.program.gyroIMU;
import org.firstinspires.ftc.robotcontroller.internal.program.linefollowingV1;
import org.firstinspires.ftc.robotcontroller.internal.program.opticaldistance;
import org.firstinspires.ftc.robotcontroller.internal.program.sensorLEGOUltrasonic;


/**
 * {@link FtcOpModeRegister} is responsible for registering opmodes for use in an FTC game.
 * @see #register(OpModeManager)
 */
public class FtcOpModeRegister implements OpModeRegister {

    /**
     * {@link #register(OpModeManager)} is called by the SDK game in order to register
     * OpMode classes or instances that will participate in an FTC game.
     *
     * There are two mechanisms by which an OpMode may be registered.
     *
     *  1) The preferred method is by means of class annotations in the OpMode itself.
     *  See, for example the class annotations in {@link ConceptNullOp}.
     *
     *  2) The other, retired,  method is to modify this {@link #register(OpModeManager)}
     *  method to include explicit calls to OpModeManager.register().
     *  This method of modifying this file directly is discouraged, as it
     *  makes updates to the SDK harder to integrate into your code.
     *
     * @param manager the object which contains methods for carrying out OpMode registrations
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.TeleOp
     * @see com.qualcomm.robotcore.eventloop.opmode.Autonomous
     */
    public void register(OpModeManager manager) {
        manager.register("Hello", TeleOpV3.class);
        manager.register("driveonly", driveOnly.class);
        //manager.register("CopOp", CopOp.class);
        //manager.register("TeleOp", CopOp.class);
        //manager.register("k9", K9botTeleopTank_Linear.class);
        //manager.register("DriveBot", DriveBotTank.class);
        manager.register("Test", Test.class);
        //manager.register("Test2", Test2.class);
        //manager.register("ArmLauncher", ArmLauncherTest.class);
        //manager.register("autonomousV1", autonomousV1.class);
        //manager.register("TestAuto2", autonomousV2.class);
        //manager.register("TestAuto3", autonomousV3.class);
        //manager.register("ballarm", ballarm.class);
        //manager.register("SensorRGB", SensorAdafruitRGB.class);
        //manager.register("linefollowingV1", linefollowingV1.class);
       // manager.register("gyroIMU", gyroIMU.class);
        //manager.register("gyroTestV1", gyroTestV1.class);
        //manager.register("gyroTestV2", gyroTestV2.class);
        //manager.register("opticaldistance", opticaldistance.class);
        //manager.register("beaconpusherV1", beaconpusherV1.class);
        //manager.register("ultrasonicSensor", sensorLEGOUltrasonic.class);
        //manager.register("ballBlueshoot", BallShootingBlue.class);
        //manager.register("ballRedshoot", BallShootingRed.class);
//        manager.register("BlueFarBeacon", BlueFarBeaconV1.class);
//        manager.register("RedFarBeacon", RedFarBeaconV1.class);
//        manager.register("BlueCloseBeacon", BlueCloseBeaconV1.class);
//        manager.register("RedCloseBeacon", RedCloseBeaconV1.class);
        //manager.register("gfg", GyroCrap.class);
        //manager.register("BSB", BlueSideBeacon.class);
        //manager.register("BWF", BlueWallFollow.class);
        manager.register("DC", DrivingChassis.class);
        //manager.register("BlueDoubleBeacon", NewDoubleBeaconProgramThatIsForWinningRegionalsBlue.class);
       // manager.register("RedDoubleBeacon", NewDoubleBeaconProgramThatIsForWinningRegionalsRed.class);
        manager.register("omnidrve", omnidrive.class);

        /**
         * Register OpModes implemented in the Blocks visual programming language.
         */
        BlocksOpMode.registerAll(manager);

        /**
         * Register OpModes that use the annotation-based registration mechanism.
         */
        AnnotatedOpModeRegistrar.register(manager);

        /**
         * Any manual OpMode class registrations should go here.
         */
    }
}
