
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

/*
    * This file contains the MoBots / MoreBots Field Centric Linear "OpMode". This is the base
    * template for both robots. Built in functionality includes: field centric motion, via
    * MoreMoBOts drivetrain creating the bones for a full Tele Op.
    *
    * Field Centric is a method of driving a robot where the robot moves in the direction of the
    * joystick regardless of the robot's orientation. This is done by rotating the joystick input
    * by the robot's heading.
    *
    * This is READ ONLY. If you want to make your own, copy this class and paste it with a new name.
    *
    *
 */

package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robomossystem.*;
import org.firstinspires.ftc.teamcode.utility.*;

@TeleOp(name="Decode Teleop", group="Decode")


public class DecodeTeleOp extends LinearOpMode {
    //get an instances of subsystem classes.
    private MoMoreBotsDrivetrain drivetrain = new MoMoreBotsDrivetrain(this);
    private intakeShooter intksht = new intakeShooter(this);
    private vision vision = new vision(this);

    // Get instance of Dashboard. Make sure update telemetry and sen packet are at end of opmode
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    public String specimenPosition = null;
    boolean dummy;
    /*TODO read alliance from singleton set by auton*/
    String alliance = null;
    // Variables for drivetrain
    double xComponent =0;
    double yComponent =0;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables. Note that the strings used here must correspond
         * to the names assigned during the robot configuration step on the DS or RC devices.
         * These names are critical, label the front of the robot as FRONT. This will be
         * important later! Don't initialize motors or IMU, they are part of the drivetrain.
         */

        //Initialize Drivetrain
        drivetrain.initialize();
        intksht.init();
        vision.init();
        intksht.stopServo();
        // Get allaince from start info
        alliance= allianceData.allianceInstance().getAlliance();

        //Ask operator which alliance they are on
        while (alliance == null){
            if (gamepad1.b) {alliance="Red";}
            if (gamepad1.x) {alliance="Blue";}
            telemetry.addLine("Please Select which Alliance");
            telemetry.addLine("Press B for Red and X for Blue");
            telemetry.update();
        }
        telemetry.addData("Selected Alliance", alliance);
        telemetry.update();


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /* Call periodic for subsystems that have a periodic voids,
             do these first as some of the voids called by the commands and other sequences
             require data from the periodic functions.
             */
            intksht.periodic();
            dummy = drivetrain.periodic(); //This is called as a variable for scheduling reasons


            //cool stuff
            telemetry.addData("Bot heading", drivetrain.heading);

            // Gamepad inputs
            // Run shooter servo on a
            if (gamepad2.a) {
                intksht.servoForward();
            } else if (gamepad2.aWasReleased()) {
                intksht.stopServo();
            }
            // Run intake on b, stop when released
            if (gamepad2.b) {
                intksht.runIntake();
            } else if (gamepad2.bWasReleased()) {
                intksht.stopIntake();
            }
            // Reverse servo and shooter on x, stop when released
            if (gamepad2.x) {
                intksht.servoReverse();
                intksht.revShooter();
            } else if (gamepad2.xWasReleased()) {
                intksht.stopServo();
                intksht.stopShooter();
            }
            // Shoot on y, stop when released
            if (gamepad2.right_trigger > 0.1) {
                intksht.shoot();
            } else if (gamepad2.right_trigger <0.1 && intksht.shooting) {
                intksht.stopShooting();
            }
            if (gamepad2.left_trigger > 0.1) {
                intksht.shootWpower(0.71);
            } else if (gamepad2.left_trigger <0.1 && intksht.shooting) {
                intksht.stopShooting();
            }
            if (gamepad2.y) {
                intksht.reverseIntake();}
            else if (gamepad2.yWasReleased()) {
                    intksht.stopIntake();
                }


            //Reset Yaw of IMU for FC drive if Driver hits back
            if (gamepad1.start) {
                drivetrain.resetIMUyaw();
            }



            /* Call Field Centric drive from drive train after calculating the speed factor
            the speed factor will be the fraction of full speed that full stick will result
            in. For example 1 is full speed, 0.5 is half speed. THe intention is to use the
            right trigger to to create a "Turbo" mode while allowing the driver to release
            the trigger and slow the robot down giving more control for small moves. a
             */
            double speedfact = 0.4;
            //If trigger pulled set speed factor to higher value
            if (gamepad1.right_trigger > 0.1) {
                speedfact = 0.9;
            }

            //Determine X and Y values to send to drivetrain based on Alliance
            if (alliance == "Blue"){
                xComponent = gamepad1.left_stick_y;
                yComponent = gamepad1.left_stick_x;
            } else {
                xComponent = gamepad1.left_stick_y * -1;
                yComponent = gamepad1.left_stick_x *-1;
            }
            //Call Field Centric void in drivetrain.
            //Commented out field centric
            // TODO Uncomment the below line to go back to robot FC, but then comment the moveRobot line
            //calculate alignment
            double alignTurn;//// = vision.tagHeading() * Constants.Drivetrain.alignGain;
            if (gamepad1.a) {
                //calculate alignment
                 alignTurn = vision.tagHeading() * Constants.Drivetrain.alignGain;
                drivetrain.moveRobotFC(xComponent, yComponent, alignTurn, speedfact);
            } else {
                drivetrain.moveRobotFC(xComponent, yComponent, gamepad1.right_stick_x, speedfact);
                // drivetrain.moveRobot(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x);
            }
                //update dashboard and telemetry if used
                if (Constants.Telemetry.showTelemetry) {
                    telemetry.addData("tagheading", vision.tagHeading());
                    telemetry.addData("tagDistance", vision.tagDistance());
                    telemetry.update();
                }
                if (Constants.Telemetry.showDashBoard) {
                    packet.put("OTOS Heading", drivetrain.otosHead);
                    packet.put("OTOS X", drivetrain.otosXPostion);
                    packet.put("OTOS Y", drivetrain.otosYPostion);
                    packet.put("Bot Heading", drivetrain.heading);
                    packet.put("Left Front Power", drivetrain.LFpower);
                    packet.put("Left Rear Power", drivetrain.LRpower);
                    packet.put("Right Front Power", drivetrain.RFpower);
                    packet.put("Right Rear Power", drivetrain.RRpower);

                    dashboard.sendTelemetryPacket(packet);
                }

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

