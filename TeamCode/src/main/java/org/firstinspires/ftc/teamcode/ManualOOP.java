package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="Manual Version", group="")
public class ManualOOP extends LinearOpMode{
    private HornetRobo hornetRobo = new HornetRobo() ;

    private LogManager logManager = new LogManager(this.telemetry, "Manual New Version");
    private ManualManager manualManager = new ManualManager(this, hornetRobo);

    private boolean isSlow = false;

    @Override
    public void runOpMode() {
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        manualManager.Init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            logManager.WriteLog("Status", "Ready to run...");


            manageDriveMotors();
            manageLauncher();
            manageRamp();
            manageIntake();

            manualManager.SetMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
            
        }
    }

    private void manageDriveMotors(){
        double drive        = 0;
        double turn         = 0;
        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = gamepad2.right_stick_x;
        turn  = gamepad2.right_stick_y;

        //if button on gamepad2 is pressed
        //then drive and turn should be half
        if (gamepad2.a){
            isSlow = true;
        }
        if (gamepad2.b){
            isSlow = false;
        }

        if (isSlow){
            drive = drive / 2;
            turn = turn / 2;
        }

        // Combine drive and turn for blended motion. Use RobotHardware class
        manualManager.DriveRobot(drive, turn);

        logManager.WriteLog("Drivexxxxx", "Left Stick");
        logManager.WriteLog("Turn", "Right Stick");
        logManager.WriteLog("-", "-------");

        logManager.WriteLog("Drive Power", Double.toString(drive));
        logManager.WriteLog("Turn Power",  Double.toString(turn));
        
    }

    private void manageStrafe() {

    }

    private void manageLauncher(){

        logManager.WriteLog("Grabber Key", Boolean.toString(gamepad1.a || gamepad1.b));
        if(gamepad1.left_stick_y > 0){
            manualManager.SetLauncherDirectionForward();
            logManager.WriteLog("Status", "Left Joystick Moved up for launcher");
            manualManager.SetLauncherPower(1);
        }
        if(gamepad1.left_stick_y < 0){
            manualManager.SetLauncherDirectionReverse();
            logManager.WriteLog("Status", "Left Joystick Moved down");
            manualManager.SetLauncherPower(1);
        }

        if(gamepad1.left_stick_y==0){
            manualManager.SetLauncherPower(0);
        }

    }

    private void manageRamp(){
        if(gamepad1.right_stick_y > 0){
            manualManager.SetRampDirectionForward();
            manualManager.SetRampPower(1);
            logManager.WriteLog("Status", "Right Joystick Moved up");
        }
        if(gamepad1.right_stick_y < 0){
            manualManager.SetRampDirectionReverse();
            logManager.WriteLog("Status", "Right Joystick Moved down");
            manualManager.SetRampPower(1);
        }
        if(gamepad1.right_stick_y==0){
            manualManager.SetRampPower(0);
        }

    }

    private void manageIntake(){
        if(gamepad1.right_trigger > 0){
            manualManager.SetRampDirectionForward();
            manualManager.SetRampPower(1);
            logManager.WriteLog("Status", "Right Joystick Moved up");
        }
        if(gamepad1.right_trigger < 0){
            manualManager.SetRampDirectionReverse();
            logManager.WriteLog("Status", "Right Joystick Moved down");
            manualManager.SetRampPower(1);
        }
        if(gamepad1.right_trigger==0){
            manualManager.SetRampPower(0);
        }

    }

}
