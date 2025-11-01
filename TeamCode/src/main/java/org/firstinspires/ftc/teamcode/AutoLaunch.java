package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Auto Launch", group="")
public class AutoLaunch extends LinearOpMode {

    static final double     DRIVE_SPEED             = 0.9;

    private HornetRobo hornetRobo;

    //manager classes
    private LauncherManager launcherManager;
    private RampManager rampManager;
    private IntakeManager intakeManager;
    private DriveManager driveManager;



    public void initialize()
    {
        hornetRobo = new HornetRobo();
        driveManager = new DriveManager(this, hornetRobo);
        HardwareMapper.MapToHardware(this, hornetRobo);
        //launcherManager = new LauncherManager(this, hornetRobo);
        //rampManager = new RampManager(this, hornetRobo);
        //intakeManager = new IntakeManager(this, hornetRobo);
    }

    public void runOpMode() {
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Starting to move", "");
            telemetry.update();

            //set forward
            //driveManager.SetMotorDirection(DriveManager.DriveDirection.FORWARD);

            while (opModeIsActive() && !isStopRequested()) {

                telemetry.addData("Move to reach submersible  ", "");
                telemetry.update();

                driveManager.MoveStraightToPosition(DriveManager.DriveDirection.FORWARD,
                        0.5,
                        1);

                driveManager.StopRobo();
                telemetry.addData("Stopped Robo", "");
                telemetry.update();

                break;
            }

        }
    }
}