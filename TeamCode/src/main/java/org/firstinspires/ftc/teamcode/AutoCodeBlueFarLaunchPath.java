package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Auto Code Blue Far Launch Path", group="")
public class AutoCodeBlueFarLaunchPath extends LinearOpMode {

    static final double     DRIVE_SPEED             = 1;

    private HornetRobo hornetRobo;

    //manager classes
    private LauncherManager launcherManager;
    private RampManager rampManager;
    private IntakeManager intakeManager;
    private DriveManager driveManager;

    private double farLaunchPower = 0.73;



    public void initialize()
    {
        hornetRobo = new HornetRobo();
        HardwareMapper.MapToHardware(this, hornetRobo);
        driveManager = new DriveManager(this, hornetRobo);
        launcherManager = new LauncherManager(this, hornetRobo);
        rampManager = new RampManager(this, hornetRobo);
        intakeManager = new IntakeManager(this, hornetRobo);
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

                driveManager.MoveStraightToPosition(DriveManager.DriveDirection.FORWARD, DRIVE_SPEED, 10);

                driveManager.TurnUsingEncoders(DriveManager.DriveDirection.LEFT, DRIVE_SPEED,6);

                Launch();

                //initial balls are launched

                driveManager.TurnUsingEncoders(DriveManager.DriveDirection.RIGHT, DRIVE_SPEED,28);

                driveManager.StrafeToPosition(DriveManager.DriveDirection.RIGHT, DRIVE_SPEED,15);

                telemetry.addData("Move to min", "");
                telemetry.update();
                intakeManager.SetDirection(DriveManager.DriveDirection.FORWARD);
                intakeManager.SetPower(1.0);
                telemetry.addData("Move to max", "");
                telemetry.update();
                sleep(750);

                driveManager.MoveStraightToPosition(DriveManager.DriveDirection.BACKWARD, DRIVE_SPEED, 45);

                intakeManager.SetPower(0.0);

                //reverses forward movement, edit to match
                driveManager.MoveStraightToPosition(DriveManager.DriveDirection.FORWARD, DRIVE_SPEED, 45);

                driveManager.StrafeToPosition(DriveManager.DriveDirection.LEFT, DRIVE_SPEED,15);

                driveManager.TurnUsingEncoders(DriveManager.DriveDirection.LEFT, DRIVE_SPEED,28);

                Launch();




                driveManager.StopRobo();
                telemetry.addData("Stopped Robo", "");
                telemetry.update();

                break;
            }

        }
    }

    public void Launch() {

        //Launcher Pre-Spinning
        telemetry.addData("Open", "");
        telemetry.update();
        launcherManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
        launcherManager.SetPower(farLaunchPower);
        telemetry.addData("Close", "");
        telemetry.update();
        sleep(1500);

        //Ramp Firing
        telemetry.addData("Move to min", "");
        telemetry.update();
        rampManager.SetDirection(DriveManager.DriveDirection.FORWARD);
        rampManager.SetPower(0.67);
        telemetry.addData("Move to max", "");
        telemetry.update();
        sleep(750);

        launcherManager.SetPower(0.0);
        rampManager.SetPower(0.0);

        //one ball fired

        //Ramp Loading
        telemetry.addData("Move to min", "");
        telemetry.update();
        rampManager.SetDirection(DriveManager.DriveDirection.FORWARD);
        rampManager.SetPower(0.57);
        telemetry.addData("Move to max", "");
        telemetry.update();
        sleep(750);


        //Launcher Pre-Spinning
        telemetry.addData("Open", "");
        telemetry.update();
        launcherManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
        launcherManager.SetPower(farLaunchPower);
        telemetry.addData("Close", "");
        telemetry.update();
        sleep(1500);

        //Intake Spinning

        telemetry.addData("Move to min", "");
        telemetry.update();
        intakeManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
        intakeManager.SetPower(0.2);
        telemetry.addData("Move to max", "");
        telemetry.update();
        sleep(100);

        telemetry.addData("Move to min", "");
        telemetry.update();
        intakeManager.SetDirection(DriveManager.DriveDirection.FORWARD);
        intakeManager.SetPower(1.0);
        telemetry.addData("Move to max", "");
        telemetry.update();
        sleep(750);

        //Ramp Firing
        telemetry.addData("Move to min", "");
        telemetry.update();
        rampManager.SetDirection(DriveManager.DriveDirection.FORWARD);
        rampManager.SetPower(0.57);
        telemetry.addData("Move to max", "");
        telemetry.update();
        sleep(750);

        launcherManager.SetPower(0.0);
        rampManager.SetPower(0.0);
        intakeManager.SetPower(0.0);
    }
}