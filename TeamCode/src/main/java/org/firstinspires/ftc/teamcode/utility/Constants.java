package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Servo;

public class Constants {
    // Create subclasses for the Drivetrain and each control item such as servos and motors.

    public static final class Drivetrain {
        // Hardware Configuration for Drivetrain Items
        // ------------------------------------------------------------

        // Drivetrain Motors, Define configured name and direction
        public static final String MOTOR_LF = "leftFrontDrive";
        public static final DcMotorSimple.Direction LF_Direction =DcMotorSimple.Direction.REVERSE;
        public static final String MOTOR_LB = "leftBackDrive";
        public static final DcMotorSimple.Direction LB_Direction =DcMotorSimple.Direction.FORWARD;
        public static final String MOTOR_RF = "rightFrontDrive";
        public static final DcMotorSimple.Direction RF_Direction =DcMotorSimple.Direction.FORWARD;
        public static final String MOTOR_RB = "rightBackDrive";
        public static final DcMotorSimple.Direction RB_Direction =DcMotorSimple.Direction.FORWARD;

        //Define name of IMU on Control hub and Control lHub Orientation Logo facing direction need to be revered depending on
        //if robot starts facing the wall or not
        public static final String IMU = "imu";
        public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_WALL = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_AWAY = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // Position Tuning constants
        // TODO Tune gains and accels for robot. Currnently moves in an odd rhomboid way.

        public static final double X_GAIN          = 0.06; //0.025;    // Strength of axial position control
        public static final double X_ACCEL         = 1000;  //previously 50   // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
        public static final double X_TOLERANCE     = 1.5;     // Controller is is "inPosition" if position error is < +/- this amount
        public static final double X_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
        public static final double X_MAX_AUTO      = 0.9;     // "default" Maximum Axial power limit during autonomous

        public static final double Y_GAIN         =  0.06; //0.025;    // Strength of lateral position control
        public static final double Y_ACCEL        = 1000;  //previously 50   // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
        public static final double Y_TOLERANCE    = 1.5;     // Controller is is "inPosition" if position error is < +/- this amount
        public static final double Y_DEADBAND     = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
        public static final double Y_MAX_AUTO     = 0.9;     // "default" Maximum Lateral power limit during autonomous

        public static final double HEADING_GAIN            = -0.04; //-0.025;    // Strength of Yaw position control
        public static final double HEADING_ACCEL           = 1000;  //previously 50   // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
        public static final double HEADING_TOLERANCE       = 3.0;     // Controller is is "inPosition" if position error is < +/- this amount
        public static final double HEADING_DEADBAND        = 0.5;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
        public static final double HEADING_MAX_AUTO        = 0.9;     // "default" Maximum Yaw power limit during autonomous

        //Constants for OTOS
        public static final String Otos ="otos"; // Define Devicename for Otos

        //Todo define starting linear and angular scalers, offset on robot and starting pose
        public static final SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0,0,180); // Offset for Otos mounting on robot
        public static final double linearScaler = 1.0;
        public static final double angularScaler = 1.0;

        public static final SparkFunOTOS.Pose2D R1 = new SparkFunOTOS.Pose2D(101,8.5,180.0); // Starting position red 1
        public static final SparkFunOTOS.Pose2D R2 = new SparkFunOTOS.Pose2D(81,8.5,180.0); // Starting position red 2
        public static final SparkFunOTOS.Pose2D R3 = new SparkFunOTOS.Pose2D(122,120.5,135.0); // Starting position red 3
        public static final SparkFunOTOS.Pose2D B1 = new SparkFunOTOS.Pose2D(41.5,8.5,180.0); // Starting position blue 1
        public static final SparkFunOTOS.Pose2D B2 = new SparkFunOTOS.Pose2D(61,8.5,180.0); // Starting position blue 2
        public static final SparkFunOTOS.Pose2D B3 = new SparkFunOTOS.Pose2D(24.5,126.0,-135); // Starting position blue 3

        public static final double alignGain = -0.05; //gain for auto align
    }
    public static final class IntakeShooter {
        // Drivetrain Motors, Define configured name and direction
        public static final String shooterMotor = "shooterMotor1";
        public static final DcMotorSimple.Direction shooterDirection =DcMotorSimple.Direction.FORWARD;

        public static final String intakeMotor = "intakeMotor";
        public static final DcMotorSimple.Direction intakeDirection =DcMotorSimple.Direction.FORWARD;

        public static final String intakeServo = "intakeServo";
        public static final Servo.Direction intkServDirec = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

        public static final double shootPower           =0.75;
        public static final double shooterBackUpPower   =-0.2;
        public static final double thresholdFactor =2200;  // This will be multiplied by the power to the shooter motor. The resulting
                                                                // number will be compared to the speed, if the shooter speed is above the value
                                                                // the servo will be allowed to advance a ball to the shooter.

        public static final double intakePower          =1.0;
        public static final double intakeRevPower       =-0.2;

        public static final double servoStop    =0.5;
        public static final double servoShoot   = 0.0;
        public static final double servoReverse =0.8;
    }

    public static final class Telemetry {
        public static final boolean showTelemetry       =true;
        public static final boolean showDashBoard       =true;
    }







}