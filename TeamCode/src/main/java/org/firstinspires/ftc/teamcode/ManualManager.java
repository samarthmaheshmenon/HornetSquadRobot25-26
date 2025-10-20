/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class ManualManager {

    public static enum MotorDirection {
        FORWARD,
        BACKWARD
    }

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // naming motors
    private HornetRobo hornetRobo;

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */

    // Define a constructor that allows the OpMode to pass a reference to itself.
            
    private LauncherManager launcherManager;
    private RampManager rampManager;
    private IntakeManager intakeManager;
    private DriveManager driveManager;


    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public ManualManager(LinearOpMode opmode, HornetRobo hornetRoboObj)  {
        myOpMode = opmode;

        hornetRobo = hornetRoboObj;

        driveManager = new DriveManager(opmode, hornetRobo);
        launcherManager = new LauncherManager(opmode, hornetRobo);
        rampManager = new RampManager(opmode, hornetRobo);
        intakeManager = new IntakeManager(opmode, hornetRobo);
    }

    public void Init()    {
        //Mapping hardware devices
        HardwareMapper.MapToHardware(myOpMode, hornetRobo);
        launcherManager.SetDirection(DriveManager.DriveDirection.FORWARD);
        rampManager.SetDirection(DriveManager.DriveDirection.FORWARD);
        intakeManager.SetDirection(DriveManager.DriveDirection.FORWARD);
        
        myOpMode.telemetry.addData("Status", "Hardware Initialized");
        myOpMode.telemetry.update();
    }


    public void SetMotorsMode(DcMotor.RunMode MotorRunMode) {

        driveManager.SetAllMotorsMode(MotorRunMode);
    }


    public boolean IsLeftMotorBusy()
    {
        return driveManager.IsMotorBusy(DriveManager.MotorPosition.LEFT_FRONT);
    }

    public boolean isRightMotorBusy()
    {
        return driveManager.IsMotorBusy(DriveManager.MotorPosition.RIGHT_FRONT);
    }

    public void SetMotorDirection(DriveManager.DriveDirection Direction){
       driveManager.SetMotorDirection(Direction);
    }

    public void ReverseMotors(){
        driveManager.SetMotorDirection(DriveManager.DriveDirection.BACKWARD);
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void DriveRobot(double Drive, double Turn) {
        // Combine drive and turn for blended motion.
        double left  = Drive + Turn;
        double right = Drive - Turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Use existing function to drive both wheels.
        driveManager.SetDrivePower(left, right);
    }

    public void GoDiagonal(double Num){
        driveManager.SetPowerToGoDiagonal(Num);
    }

    public void GoStrafe (double Num){
        driveManager.SetPowerToStrafe(Num);
    }

    public void SetLauncherMode (DcMotor.RunMode Mode){
       launcherManager.SetMotorsMode(Mode);
    }

    public void SetLauncherPower (double Power){
        launcherManager.SetPower(Power);
    }

    public void SetLauncherDirectionForward() {
        launcherManager.SetDirection(DriveManager.DriveDirection.FORWARD);
    }

    public void SetLauncherDirectionReverse() {
        launcherManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
    }

    public void SetRampMode (DcMotor.RunMode Mode){
        rampManager.SetMotorsMode(Mode);
    }

    public void SetRampPower (double Power){
        rampManager.SetPower(Power);
    }

    public void SetRampDirectionForward() {
        rampManager.SetDirection(DriveManager.DriveDirection.FORWARD);
    }

    public void SetRampDirectionReverse() {
        rampManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
    }

    public void SetIntakeMode (DcMotor.RunMode Mode){
        intakeManager.SetMotorsMode(Mode);
    }

    public void SetIntakePower (double Power){
        intakeManager.SetPower(Power);
    }

    public void SetIntakeDirectionForward() {
        intakeManager.SetDirection(DriveManager.DriveDirection.FORWARD);
    }

    public void SetIntakeDirectionReverse() {
        intakeManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
    }
}


