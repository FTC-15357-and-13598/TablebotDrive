/* Created 9/12/2024 Brian Herioux
   Contiains Drivetrian code using SparkFun OTOS odemetry snesor
   Drivetrain cod used for MoBots and MoreBots
*/

package org.firstinspires.ftc.teamcode.robomossystem;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utility.Constants;

import java.util.List;

public class MoMoreBotsDrivetrain {
    // Public Members
    public double heading           = 0;
    public double IMU_Radians;
    public double otosXPostion;
    public double otosYPostion;
    public double otosHead;
    public double otosRad;
    public double RFpower, RRpower,LFpower,LRpower;

    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    public ProportionalControl2 xController     = new ProportionalControl2(Constants.Drivetrain.X_GAIN, Constants.Drivetrain.X_ACCEL,
            Constants.Drivetrain.X_MAX_AUTO, Constants.Drivetrain.X_TOLERANCE, Constants.Drivetrain.X_DEADBAND, false);
    public ProportionalControl2 yController    = new ProportionalControl2(Constants.Drivetrain.Y_GAIN, Constants.Drivetrain.Y_ACCEL,
            Constants.Drivetrain.Y_MAX_AUTO, Constants.Drivetrain.Y_TOLERANCE, Constants.Drivetrain.Y_DEADBAND, false);
    public ProportionalControl2 headingController = new ProportionalControl2(Constants.Drivetrain.HEADING_GAIN, Constants.Drivetrain.HEADING_ACCEL,
            Constants.Drivetrain.HEADING_MAX_AUTO, Constants.Drivetrain.HEADING_TOLERANCE,Constants.Drivetrain.HEADING_DEADBAND, true);

    //SparkfunOtos is myOtos
    SparkFunOTOS myOtos;
    private final int READ_PERIOD = 1;

    // ---  Private Members

    // Hardware interface Objects
    private DcMotor leftFrontDrive;     //  control the left front drive wheel
    private DcMotor rightFrontDrive;    //  control the right front drive wheel
    private DcMotor leftBackDrive;      //  control the left back drive wheel
    private DcMotor rightBackDrive;     //  control the right back drive wheel

    /* // FTC Dashboard - Access at 192.168.43.1:8080/dash - See packets later on in the code
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();*/

    private LinearOpMode myOpMode;
    private IMU imu;
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    private double previousOtosHeading = 0;
    private double previousTime = 0;

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU
    private double otosTurn           = 0; // Latest Robot Turn Rate from OTOS
    private boolean showTelemetry     = true; // set to true to display telemetry

    boolean skipsetinit =false;

    // Robot Constructor
    public MoMoreBotsDrivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

     /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     *
     */
    public void initialize()
    {
        //Select start position variables to be use to initialize IMU and Otos
        // define variable as 4 if start position is 1-3 it will be changed below
        RevHubOrientationOnRobot imuOrientation= new RevHubOrientationOnRobot(Constants.Drivetrain.HUB_LOGO_AWAY,Constants.Drivetrain.HUB_USB);
        // Initialize the hardware variables. Note that the strings used to 'get' each
        // motor/device must match the names assigned during the robot configuration.

        // !!!  Set the drive direction to ensure positive power drives each wheel forward.
        // !!! BadMonkey has a reversed motor. Hence the odd forward/reverse below
        leftFrontDrive  = setupDriveMotor(Constants.Drivetrain.MOTOR_LF, Constants.Drivetrain.LF_Direction);
        rightFrontDrive = setupDriveMotor(Constants.Drivetrain.MOTOR_RF, Constants.Drivetrain.RF_Direction);
        leftBackDrive  = setupDriveMotor(Constants.Drivetrain.MOTOR_LB, Constants.Drivetrain.LB_Direction);
        rightBackDrive = setupDriveMotor(Constants.Drivetrain.MOTOR_RB, Constants.Drivetrain.RB_Direction);
        imu = myOpMode.hardwareMap.get(IMU.class, Constants.Drivetrain.IMU);

        // Connect to the OTOS
        myOtos = myOpMode.hardwareMap.get(SparkFunOTOS.class,Constants.Drivetrain.Otos);
        myOtos.setLinearUnit(DistanceUnit.INCH); //Units are inches
        myOtos.setAngularUnit(AngleUnit.DEGREES); //And in degrees
        myOtos.setOffset(Constants.Drivetrain.offset); //This tells the Otos where it is mounted on the robot
        myOtos.setLinearScalar(Constants.Drivetrain.linearScaler); //This sets the linear scalar to correct distance measurement
        myOtos.setAngularScalar(Constants.Drivetrain.angularScaler); //This sets the angular scalar to correct angle reading.
        myOtos.calibrateImu(255, false); //Always calibrate the IMU

        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Tell the software how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly based on case above

        imu.initialize(new IMU.Parameters(imuOrientation));
        imu.resetYaw();
    }

    public void setPose (SparkFunOTOS.Pose2D pose) {
        myOtos.resetTracking(); //This resets the tracking of the sensor
        myOtos.setPosition(pose); //This sets the position of the sensor
    }

    /**
     *   Setup a drive motor with passed parameters.  Ensure encoder is reset.
     * @param deviceName  Text name associated with motor in Robot Configuration
     * @param direction   Desired direction to make the wheel run FORWARD with positive power input
     * @return the DcMotor object
     */
    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    /**
     * Read all input devices to determine the robot's motion and all other
     * periodic functions of this subsystem. It needs to be called by the
     * Auton or Teliop.
     */
    public boolean periodic () {
        double currentTime = myOpMode.getRuntime();
        otosXPostion = myOtos.getPosition().x;   // To send to dashboard
        otosYPostion = myOtos.getPosition().y;   // To send to dashboard
        otosHead = myOtos.getPosition().h;       // To send to dashboard
        otosRad = Math.toRadians(otosHead);      //Used for FC Auton control in gotoPostition
        RFpower =rightFrontDrive.getPower();     // To send to dashboard
        RRpower = rightBackDrive.getPower();     // To send to dashboard
        LFpower = leftFrontDrive.getPower();     // To send to dashboard
        LRpower = leftBackDrive.getPower();      // To send to dashboard

        // Calculate angular velocity from OTOS heading
        double deltaTime = currentTime - previousTime;
        if (deltaTime > 0) {
            otosTurn = (otosHead - previousOtosHeading) / deltaTime;
        }

        // Update previous heading and time
        previousOtosHeading = otosHead;
        previousTime = currentTime;

        //Get IMU heading, used for FC teleop drive
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        heading  = orientation.getYaw(AngleUnit.DEGREES);
        IMU_Radians =orientation.getYaw(AngleUnit.RADIANS);
        turnRate    = angularVelocity.zRotationRate;

        // Big Telemetry block to show all the values. myOpMode is for the Driver Station and packet.put is for the Dashboard.

        if (Constants.Telemetry.showTelemetry) {myOpMode.telemetry.addData("OTOS Heading", "%5.2f", otosHead);
         myOpMode.telemetry.addData("OTOS X: Y:", "%5.2f %5.2f", otosXPostion, otosYPostion);
         myOpMode.telemetry.addData("OTOS Heading", "%5.2f", otosHead);
         myOpMode.telemetry.addData("OTOS TurnRate", "%5.2f", otosTurn);
         myOpMode.telemetry.addData("imu turn rate", turnRate);
         }

        return true;
    }

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * distanceInches is used to calculate finished position which is fed to gotoPosition to complete the move
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void drive(double distanceInches, double power, double holdTime) {
        double dX,dY;
        //Calculate new position on field
        dX = otosXPostion + distanceInches*Math.cos(Math.toRadians(otosHead));
        dY = otosYPostion + distanceInches*Math.sin(Math.toRadians(otosHead));

        // goto newly calculated position
        gotoPosition(dX,dY,otosHead,power,holdTime);
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param distanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void strafe(double distanceInches, double power, double holdTime) {
        double dX,dY,angle;
        if (otosHead<= 90){
             // shooterDirection of travel is 90 greater than current heading
            angle=otosHead+90;
            }
        else {
                //Handle when 90 degrees grater than heading exceeds 180
                angle =otosHead-270;
            }
        dX = otosXPostion + distanceInches*Math.cos(Math.toRadians(angle));
        dY = otosYPostion + distanceInches*Math.sin(Math.toRadians(angle));
        // goto newly calculated position
        gotoPosition(dX,dY,otosHead,power,holdTime);
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {
        // Call gotoPosition using new heading, power and holdtime with existing x and y
        gotoPosition(otosXPostion,otosYPostion,headingDeg,power,holdTime);
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param xDistanceInches  Distance to travel.  +ve = fordward, -ve = reverse.
     * @param yDistanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */

    public void driveXY(double xDistanceInches, double yDistanceInches, double power, double holdTime) {
        double dX,dY,angle;
        //Calculate drive portion of new position on field
        dX = otosXPostion + xDistanceInches*Math.cos(Math.toRadians(otosHead));
        dY = otosYPostion + xDistanceInches*Math.sin(Math.toRadians(otosHead));
        //Calculate Strafe portion of new distance on field
        if (otosHead<= 90){
            // shooterDirection of travel is 90 greater than current heading
            angle=otosHead+90;
        }
        else {
            //Handle when 90 degrees grater than heading exceeds 180
            angle =otosHead-270;
        }
        dX = dX + otosXPostion + yDistanceInches*Math.cos(Math.toRadians(angle));
        dY = dY + otosYPostion + yDistanceInches*Math.sin(Math.toRadians(angle));

        // goto newly calculated position
        gotoPosition(dX,dY,otosHead,power,holdTime);

    }
    /**
     * Goto specific position and heading on field called by AUTON
     * @param xPostion  Distance to travel.  +ve = forward, -ve = reverse.
     * @param yPostion Maximum power to apply.  This number should always be positive.
     * @param direction final heading robot is to be pointing
     * @param power  Max power for postion controller to use
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void gotoPosition(double xPostion, double yPostion, double direction, double power,double holdTime) {

        xController.reset(xPostion, power);   // achieve desired X position
        yController.reset(yPostion, power);  // achieve desired Y position
        headingController.reset(direction, power);    // achieve desired heading
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && periodic()){

            // implement desired axis powers
            moveRobotFCAuto(xController.getOutput(otosXPostion), yController.getOutput(otosYPostion), headingController.getOutput(otosHead));
            //moveRobotFCAuto(yController.getOutput(otosYPostion), xController.getOutput(otosXPostion), headingController.getOutput(otosHead));


            myOpMode.telemetry.addData("x output",xController.getOutput(otosXPostion));
            myOpMode.telemetry.addData("y output",yController.getOutput(otosYPostion));
            myOpMode.telemetry.addData("h output",headingController.getOutput(otosHead));
            myOpMode.telemetry.addData("x position",otosXPostion);
            myOpMode.telemetry.addData("y position",otosYPostion);
            myOpMode.telemetry.addData("In Pos x",xController.inPosition());
            myOpMode.telemetry.addData("In Pos y",yController.inPosition());
            myOpMode.telemetry.addData("In Pos h",headingController.inPosition());
            myOpMode.telemetry.addData("h error",headingController.setPoint);
            myOpMode.telemetry.update();

            // Time to exit?
            if (xController.inPosition() && yController.inPosition() && headingController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }

        if (holdTime>0.1) {
            stopRobot(); // Only stop robot if there is a hold time
        }

    }


    //  ########################  Low level control functions.  ###############################

    /**
     * Drive the wheel motors to obtain the requested axes motions
     * @param drive     Fwd/Rev axis power
     * @param strafe    Left/Right axis power
     * @param yaw       Yaw axis power
     */
    public void moveRobot(double drive, double strafe, double yaw){

        //Added a .25 multiplier for slow speed in robot centric mode.

        double lF = (drive - strafe - yaw) * 0.25;
        double rF = (drive + strafe + yaw) * 0.25;
        double lB = (drive + strafe - yaw) * 0.25;
        double rB = (drive - strafe + yaw) * 0.25;

        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));

        //normalize the motor values
        if (max > 1.0)  {
            lF /= max;
            rF /= max;
            lB /= max;
            rB /= max;
        }

        //send power to the motors
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);

        /*if (showTelemetry) {
            myOpMode.telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
            myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
            myOpMode.telemetry.update(); //  Assume this is the last thing done in the loop.
        }*/
    }
    /**
     * Field Centric Drive to be called by teliop with inputs from driver's controller
     * @param leftX    Fwd/Rev axis power
     * @param leftY    Left/Right axis power
     * @param rightX       Yaw axis power
     * @param turbofactor   Speed factor for teliop 1= full speed make <1 to slow down
     */
    public void moveRobotFC(double leftX, double leftY, double rightX, double turbofactor){

         //Calculate rotX and rotY for use in motor output calcs
        double rotX = leftX * Math.cos(-otosRad) - leftY * Math.sin(-otosRad);  //was -IMU_Radians
        double rotY = leftX * Math.sin(-otosRad) + leftY * Math.cos(-otosRad);  //was -IMU_Radians

        /*
         Denominator is the largest motor power (absolute value) or 1 and manipulated by
         the trubofactor to give the driver more control in close quarters.
         This ensures all the powers maintain the same ratio, but only when
         at least one is out of the range [-1, 1]
         insure 1.0<=turbofactor<=0.2 to not overdrive or go too slow in case of error
         in calling function. turbofactor is set to 1 if greater than 1, 0.2 if less
         than 0.2.
        */
        turbofactor = Range.clip(turbofactor,0.2,1.0);

        double denominator = Math.max(Math.abs(leftY) + Math.abs(leftX) + Math.abs(rightX), 1);
        double lF = ((rotY + rotX + rightX) / denominator) * turbofactor;
        double lB = ((rotY - rotX + rightX) / denominator) * turbofactor;
        double rF = ((rotY - rotX - rightX) / denominator) * turbofactor;
        double rB = ((rotY + rotX - rightX) / denominator) * turbofactor;

        //send power to the motors
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);

    }

    /**
     * Field Centric Drive to be called by Auton to drive to position read by OTOS.
     * The only difference between this and moveRobotFC is that this function uses
     * the heading from the OTOS rather than the IMU
     * @param X     X axis power
     * @param Y     Y axis power
     * @param rot   Rotation axis power
     */
    public void moveRobotFCAuto(double X, double Y, double rot){

        //Calculate rotX and rotY for use in motor output calcs
        double rotX = X * Math.cos(-otosRad) - Y * Math.sin(-otosRad);
        double rotY = X * Math.sin(-otosRad) + Y * Math.cos(-otosRad);

        /*
         Denominator is the largest motor power (absolute value) or 1
         This ensures all the powers maintain the same ratio, but only when
         at least one is out of the range [-1, 1]
        */

        double denominator = Math.max(Math.abs(Y) + Math.abs(X) + Math.abs(rot), 1);
        double lF = ((rotY + rotX + rot) / denominator);
        double lB = ((rotY - rotX + rot) / denominator);
        double rF = ((rotY - rotX - rot) / denominator);
        double rB = ((rotY + rotX - rot) / denominator);

        //send power to the motors
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);

    }

    /**
     * Stop all motors.
     */
    public void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**
     * Set odometry counts and distances to zero.
     */
    /*public void resetOdometry() {

        //myOtos.resetTracking; // TODO Moved from the end. 9/3/2024
        //myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        driveOdometerOffset = rawDriveOdometer;
        driveDistance = 0.0;
        driveController.reset(0);

        strafeOdometerOffset = rawStrafeOdometer;
        strafeDistance = 0.0;
        strafeController.reset(0);

    }*/


    //Neither of these are used it AUTON.
    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    //Used by teliop for on-demand reset of yaw
    public void resetIMUyaw() {
        imu.resetYaw();
        myOtos.setPosition(new SparkFunOTOS.Pose2D(otosXPostion,otosYPostion,0));
    }
}

//****************************************************************************************************
//****************************************************************************************************

/***
 * This class is used to implement a proportional controller which can calculate the desired output power
 * to get an axis to the desired setpoint value.
 * It also implements an acceleration limit, and a max power output.
 */
class ProportionalControl2 {
    double  lastOutput;
    double  gain;
    double  accelLimit;
    double  defaultOutputLimit;
    double  liveOutputLimit;
    double  setPoint;
    double  tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    ElapsedTime cycleTime = new ElapsedTime();

    public ProportionalControl2(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.defaultOutputLimit = outputLimit;
        this.liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        reset(0.0);
    }

    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input  Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double input) {
        double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;
        double output;

        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }

        inPosition = (Math.abs(error) < tolerance);

        // Prevent any very slow motor output accumulation
        if (Math.abs(error) <= deadband) {
            output = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            output = (error * gain);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((output - lastOutput) > dV) {
                output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
                output = lastOutput - dV;
            }
        }

        lastOutput = output;
        cycleTime.reset();
        return output;
    }

    public boolean inPosition(){
        return inPosition;
    }
    public double getSetpoint() {return setPoint;}

    /**
     * Saves a new setpoint and resets the output power history.
     * This call allows a temporary power limit to be set to override the default.
     * @param setPoint
     * @param powerLimit
     */
    public void reset(double setPoint, double powerLimit) {
        liveOutputLimit = Math.abs(powerLimit);
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * @param setPoint
     */
    public void reset(double setPoint) {
        liveOutputLimit = defaultOutputLimit;
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Leave everything else the same, Just restart the acceleration timer and set output to 0
     */
    public void reset() {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
    }

}