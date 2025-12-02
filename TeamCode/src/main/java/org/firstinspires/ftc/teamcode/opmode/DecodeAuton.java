
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robomossystem.MoMoreBotsDrivetrain;
import org.firstinspires.ftc.teamcode.robomossystem.allianceData;
import org.firstinspires.ftc.teamcode.robomossystem.intakeShooter;
import org.firstinspires.ftc.teamcode.robomossystem.vision;
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
             during drivetrain.gotoPosition commands due to the fact they do not return until
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
                drivetrain.gotoPosition(87,85.5,135,.3,0.2);
                step =2;   //step 200 to end Auton
                break;  // every case must end with a break

            case 2:    //start shooter and wait for it to get to speed
                intksht.shootWpower (Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 3; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 3:    //at speed will turn off when first ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 4; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 4:    //will get to speed before 2nd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    intksht.runIntake();
                    step = 5; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 5:    //at speed will turn off when second ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 6; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 6:    //will get to speed before 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 7; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 7:    //at speed will turn off when 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    intksht.stopShooting();
                    intksht.stopIntake();
                    step = 8; // 3rd ball gone, move on
                }
                break;

            case 8:
                drivetrain.gotoPosition(100,85.5,-105,.3,0.2);
                intksht.runIntake();
                drivetrain.gotoPosition(119,80,-105,.2,0.2);
                intksht.stopIntake();
                drivetrain.gotoPosition(87,85.5,135,.3,0.2);
                step =9;
                break;  // every case must end with a break

            case 9:    //start shooter and wait for it to get to speed
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 10; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 10:    //at speed will turn off when first ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 11; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 11:    //will get to speed before 2nd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    intksht.runIntake();
                    step = 12; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 12:    //at speed will turn off when second ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 13; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 13:    //will get to speed before 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 14; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 14:    //at speed will turn off when 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    intksht.stopShooting();
                    intksht.stopIntake();
                    step = 200; // 3rd ball gone, move on
                }
                break;

            case 21:    //first step of Auton for Red 2
                drivetrain.gotoPosition(83,15,155,.3,.5);
               step = 22;
                break;  // every case must end with a break

            case 22:    //start shooter and wait for it to get to speed
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 23; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 23:    //at speed will turn off when first ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 24; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 24:    //will get to speed before 2nd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    intksht.runIntake();
                    step = 25; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 25:    //at speed will turn off when second ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 26; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 26:    //will get to speed before 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 27; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 27:    //at speed will turn off when 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    intksht.stopShooting();
                    intksht.stopIntake();
                    step = 28; // 3rd ball gone, move on
                }
                break;  // every case must end with a break
            case 28:
                drivetrain.gotoPosition(99,29,-55,.3,0.5);
                intksht.runIntake();
                drivetrain.gotoPosition(114,29,-55,.3,0.5);
                intksht.stopIntake();
                step = 29;

                break;
            case 29:    //first step of Auton for Red 2
                drivetrain.gotoPosition(83,15,155,.3,.5);
                step = 30;
                break;  // every case must end with a break

            case 30:    //start shooter and wait for it to get to speed
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 31; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 31:    //at speed will turn off when first ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 32; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 32:    //will get to speed before 2nd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    intksht.runIntake();
                    step = 33; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 33:    //at speed will turn off when second ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 34; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 34:    //will get to speed before 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 35; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 35:    //at speed will turn off when 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    intksht.stopShooting();
                    intksht.stopIntake();
                    step = 200; // 3rd ball gone, move on
                }
                break;  // every case must end with a break

            case 41:    //first step of Auton for Red 3
                drivetrain.gotoPosition(87,85.5,135,.3,0.2);

                step =42;
                break;  // every case must end with a break

            case 42:    //start shooter and wait for it to get to speed
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 43; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 43:    //at speed will turn off when first ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 44; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 44:    //will get to speed before 2nd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    intksht.runIntake();
                    step = 45; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 45:    //at speed will turn off when second ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 46; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 46:    //will get to speed before 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 47; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 47:    //at speed will turn off when 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    intksht.stopShooting();
                    intksht.stopIntake();
                    step = 48; // 3rd ball gone, move on
                }
                break;

            case 48:
                drivetrain.gotoPosition(100,85.5,-105,.3,0.2);
                intksht.runIntake();
                drivetrain.gotoPosition(125,80,-105,.2,0.2);
                intksht.stopIntake();
                drivetrain.gotoPosition(87,85.5,135,.3,0.2);
                step =49;
                break;  // every case must end with a break

            case 49:    //start shooter and wait for it to get to speed
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 50; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 50:    //at speed will turn off when first ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 51; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 51:    //will get to speed before 2nd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    intksht.runIntake();
                    step = 52; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 52:    //at speed will turn off when second ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 53; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 53:    //will get to speed before 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 54; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 54:    //at speed will turn off when 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    intksht.stopShooting();
                    intksht.stopIntake();
                    step = 200; // 3rd ball gone, move on
                }
                break;

            case 61:    //first step of Auton for Blue 1
                drivetrain.gotoPosition(41,60,180,.3,0);
                drivetrain.gotoPosition(58,87,-135,.3,5);
                step =200;
                break;  // every case must end with a break

            case 81:    //first step of Auton for Blue 2
                drivetrain.gotoPosition(57,14.5,-159,.3,.15);
                step = 82; // uncomment this to move to next step
                break;  // every case must end with a break

            case 82:    //start shooter and wait for it to get to speed
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 83; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 83:    //at speed will turn off when first ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 84; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 84:    //will get to speed before 2nd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    intksht.runIntake();
                    step = 85; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 85:    //at speed will turn off when second ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 86; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 86:    //will get to speed before 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 87; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 87:    //at speed will turn off when 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    intksht.stopShooting();
                    intksht.stopIntake();
                    step = 88; // 3rd ball gone, move on
                }
                break;  // every case must end with a break

            case 88:    //at speed will turn off when 3rd ball shoots
                drivetrain.gotoPosition(42,30.5,57,.4,0);
                intksht.runIntake();
                drivetrain.gotoPosition(18,30.5,57,.25,1);
                intksht.stopIntake();
                drivetrain.gotoPosition(57,14.5,-159,.3,1);
                step=89;   //End Auton
                break;  // every case must end with a break

            case 89:    //start shooter and wait for it to get to speed
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 90; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 90:    //at speed will turn off when first ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 91; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 91:    //will get to speed before 2nd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    intksht.runIntake();
                    step = 92; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 92:    //at speed will turn off when second ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 93; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 93:    //will get to speed before 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 94; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 94:    //at speed will turn off when 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonFarShot); //start shooter
                if (!intksht.shootAtSpd) {
                    intksht.stopShooting();
                    intksht.stopIntake();
                    step = 200; // 3rd ball gone, move on
                }
                break;

            case 101:    //first step of Auton for Blue 3
                drivetrain.gotoPosition(54.5,96,-135,.3,1);
                step=102;
                break;  // every case must end with a break  // every case must end with a break

            case 102:    //start shooter and wait for it to get to speed
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    wait(0.5);
                    step = 103; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 103:    //at speed will turn off when first ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 104; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 104:    //will get to speed before 2nd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    intksht.runIntake();
                    step = 105; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 105:    //at speed will turn off when second ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    step = 106; // uncomment this to move to next step
                    wait(0.5);
                }
                break;  // every case must end with a break

            case 106:    //will get to speed before 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (intksht.shootAtSpd) {
                    step = 107; // uncomment this to move to next step
                }
                break;  // every case must end with a break

            case 107:    //at speed will turn off when 3rd ball shoots
                intksht.shootWpower(Constants.IntakeShooter.autonCloseShot); //start shooter
                if (!intksht.shootAtSpd) {
                    intksht.stopShooting();
                    intksht.stopIntake();
                    step = 108; // 3rd ball gone, move on
                }
                break;
            case 108:    //first step of Auton for Blue 3
                drivetrain.gotoPosition(54.5,80,-135,.3,1);
                step=200;
                break;  // every case must end with a break  // every case must end with a break

            case 200:
                break;
        }




        telemetry.addData("Current Step", step);
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
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

