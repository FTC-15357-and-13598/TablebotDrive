
/* Copyright (c) 2021 FIRST. All rights reserved.
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
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
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

package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robomossystem.MoMoreBotsDrivetrain;
import org.firstinspires.ftc.teamcode.robomossystem.*;
import org.firstinspires.ftc.teamcode.utility.Constants;

import java.util.Objects;

@Autonomous(name="Decode Auton", group="Decode", preselectTeleOp = "Decode Teleop")


public class DecodeAuton extends LinearOpMode {
    //get an instances of subsystem classes.
    private MoMoreBotsDrivetrain drivetrain = new MoMoreBotsDrivetrain(this);
    private intakeShooter intksht= new intakeShooter(this);
    private vision vision = new vision(this);

    // Get instance of Dashboard. Make sure update telemetry and sen packet are at end of opmode
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    boolean dummy;

        String alliance = null;  // Will be written to alliance data
        int startingPosition =0; // will prompt for starting position between 1 and 3

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables. Note that the strings used here must correspond
         * to the names assigned during the robot configuration step on the DS or RC devices.
         * These names are critical, label the front of the robot as FRONT. This will be
         * important later! Don't initialize motors or IMU, they are part of the drivetrain.
         */


        //Ask operator which alliance they are on
        while (alliance == null){
            if (gamepad1.b) {alliance="Red";}
            if (gamepad1.x) {alliance="Blue";}
            telemetry.addLine("Please Select which Alliance");
            telemetry.addLine("Press B for Red and X for Blue");
            telemetry.update();
        }

        //Ask Operator which starting position the Robot is in
        while (startingPosition==0){
            if (gamepad1.dpad_up){startingPosition=1;}
            if (gamepad1.dpad_right){startingPosition=2;}
            if (gamepad1.dpad_down){startingPosition=3;}
            telemetry.addLine("Please Select Starting Position");
            telemetry.addLine("Press DPad Up for 1");
            telemetry.addLine("Press DPad Right for 2");
            telemetry.addLine("Press DPad Down for 3");
            telemetry.update();
        }

        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Starting Position", startingPosition);
        telemetry.update();

        allianceData.allianceInstance().setAlliance(alliance);

        //Initialize subsystems
        drivetrain.initialize();
        intksht.init();
        vision.init();
        intksht.stopServo();

        waitForStart();

        // Set pose of Drivetrain based on Alliance an Starting Position
        if (Objects.equals(alliance, "Blue")){
            if (startingPosition == 1) {
                drivetrain.setPose(Constants.Drivetrain.B1);
            } else if (startingPosition ==2 ) {
                drivetrain.setPose(Constants.Drivetrain.B2);
            } else {
                drivetrain.setPose(Constants.Drivetrain.B3);
            }
        } else {
            if (startingPosition == 1) {
                drivetrain.setPose(Constants.Drivetrain.R1);
            } else if (startingPosition ==2 ) {
                drivetrain.setPose(Constants.Drivetrain.R2);
            } else {
                drivetrain.setPose(Constants.Drivetrain.R3);
            }
        }
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /* Call periodic for subsystems that have a periodic voids,
             do these first as some of the voids called by the commands and other sequences
             require data from the periodic functions.
             */
            intksht.periodic();
            dummy =drivetrain.periodic(); //This is called as a variable for scheduling reasons






        telemetry.update();
        }
    }
    //Put any super-system type voids here
    //TODO add timers in place of sleep

    private ElapsedTime holdingTimer = new ElapsedTime();

    void wait(double holdTime) {
        holdingTimer.reset();
        while (holdingTimer.time() < holdTime) {
        }
    }
}

