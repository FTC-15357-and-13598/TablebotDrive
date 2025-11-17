
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
        int step   =0;  // this will be ste based on alliance and starting position

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
                step = 61;  // 61= Blue 1 auton
            } else if (startingPosition ==2 ) {
                drivetrain.setPose(Constants.Drivetrain.B2);
                step = 81; //81=Blue 2 auton
            } else {
                drivetrain.setPose(Constants.Drivetrain.B3);
                step = 101; // 101 = blue 3 auton
            }
        } else {
            if (startingPosition == 1) {
                drivetrain.setPose(Constants.Drivetrain.R1);
                step=1; // 1= red 1 auton
            } else if (startingPosition ==2 ) {
                drivetrain.setPose(Constants.Drivetrain.R2);
                step =21; // 21= Red 2 auton
            } else {
                drivetrain.setPose(Constants.Drivetrain.R3);
                step =41; // 01 = Red 3 Auton
            }
        }
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /* Call periodic for subsystems that have a periodic voids,
             do these first as some of the voids called by the commands and other sequences
             require data from the periodic functions. NOTE: Periodics will not be called
             during divetrain.gotoPosition commands due to the fact they do not return unitl
             the robot is in position.
             */
            intksht.periodic();
            dummy =drivetrain.periodic(); //This is called as a variable for scheduling reasons

            /*Steps for Auton,  initialization, step will be set to 1 if the robot is
              in position red 1, 21 if red 2, 41 if red 3, 61 if blue 1, 81, if blue 2
              and 101 if blue 3
             */

        //Begin switch structure
        switch (step){
            case 1:     //first step of Auton for Red 1
                drivetrain.gotoPosition(40.5,80,180,.3,0);
                drivetrain.gotoPosition(40.5,110,110,.3,5);
                step =200;   //step 200 to end Auton
                // step =step+1 // uncomment this to move to next step

            case 21:    //first step of Auton for Red 2
                // step =step+1 // uncomment this to move to next step

            case 41:    //first step of Auton for Red 3
                // step =step+1 // uncomment this to move to next step

            case 61:    //first step of Auton for Blue 1
                // step =step+1 // uncomment this to move to next step

            case 81:    //first step of Auton for Blue 2
                // step =step+1 // uncomment this to move to next step

            case 101:    //first step of Auton for Blue 3
                // step =step+1 // uncomment this to move to next step


            case 200:
                break;
        }




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

