/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Auto Code Test", group="")
public class AutoCodeTest extends LinearOpMode {
    static final double     DRIVE_SPEED             = 0.3;
    static final double     DRIVE_INCREASED_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.2;

    private HornetRobo hornetRobo;

    //manager classes
    private DriveManager driveManager;
    private IntakeManager intakeManager;
    private RampManager rampManager;
    private LauncherManager launcherManager;

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

        int rotateToPosition = 5;
        //driveManager.SetMotorDirection(AutoDriveManager.DriveDirection.FORWARD);
        //driveManager.TurnUsingEncoders(AutoDriveManager.DriveDirection.LEFT, DRIVE_SPEED, rotateToPosition);

        //Change motor to test all
        //driveManager.SetMotorDirection(AutoDriveManager.DriveDirection.FORWARD);
        //TestViperSlideUpOnButtonClick();

        //Test strafe
        //TestStrafeRoboUsingEncoders();

        TestLogManager();

        //Test turn
        //TestRotate180();

        //Test Move Forward

       // driveManager.MoveStraightToPosition(AutoDriveManager.DriveDirection.FORWARD, DRIVE_SPEED, 10);
       // driveManager.MoveStraightToPosition(AutoDriveManager.DriveDirection.BACKWARD, DRIVE_SPEED, 10);
/*
        TestDriveMotorEncodedMove(hornetRobo.RightFrontMotor);
        TestDriveMotorEncodedMove(hornetRobo.LeftFrontMotor);
        TestDriveMotorEncodedMove(hornetRobo.LeftBackMotor);
        TestDriveMotorEncodedMove(hornetRobo.RightBackMotor);

*/
        //Test go fwd
        //TestGoForwardRoboUsingEncoders();

        //Test Strafe
        //TestStrafeRoboUsingEncoders();

        //Test Arm
        //TestArm();

        //Test Grabber
        //TestGrabberOpenAndClose();

        //Test viper slide
        //TestViperSlide();
        //TestRotate180();

        //TestViperSlideUpOnButtonClick(); //test encoder
        //TestViperSlideWithBrake(); // test specimen hanging
    }

    public void TestDriveMotorEncodedMove(DcMotor Motor) {
        double testDistance = 5;
        DriveManager driveManager = new DriveManager(this, hornetRobo);
        int encodedDistance = driveManager.getEncodedDistance(testDistance);

        if (opModeIsActive()) {
            telemetry.addData("Starting to move", "");
            telemetry.update();

            Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive() && !isStopRequested()) {
                int currentPosition = Motor.getCurrentPosition();
                telemetry.addData("Current position: ", currentPosition);
                int newPosition = currentPosition + encodedDistance;
                telemetry.addData("New position: ", newPosition);
                Motor.setTargetPosition(newPosition);
                //Motor.setDirection(DcMotorSimple.Direction.FORWARD);
                Motor.setPower(0.3);
                //sleep(2000);
                Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.update();

                while (opModeIsActive()  && Motor.isBusy())
                {

                }
                break;

            }

        }
    }

    public void TestGoForwardRoboUsingEncoders(){
        //AutoDriveManager driveManager = new AutoDriveManager(this, hornetRobo);

        if (opModeIsActive()) {

            telemetry.addData("Starting to move", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("go forward", "");
                telemetry.update();

                driveManager.MoveStraightToPosition(DriveManager.DriveDirection.BACKWARD, DRIVE_SPEED, 8);

                telemetry.addData("go forward", "");
                telemetry.update();

                break;
            }

        }
    }

    public void TestStrafeRoboUsingEncoders() {
        DriveManager driveManager = new DriveManager(this, hornetRobo);

        if (opModeIsActive()) {
            driveManager.SetAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveManager.SetAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Starting to move", "");
            telemetry.update();

            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("go forward", "");
                telemetry.update();

                // Move 5 inches forward
                driveManager.StrafeToPosition(DriveManager.DriveDirection.LEFT, DRIVE_SPEED, 5);

                telemetry.addData("go forward", "");
                telemetry.update();

                break;
            }

        }
    }

    public void TestLogManager(){
        LogManager logManager = new LogManager(this.telemetry, "AutoCodeTest");
        logManager.WriteLog("Test1", "Test2");
        logManager.WriteLog("Test3", "Test4");
    }

    public void TestLauncher() {
         if (opModeIsActive()) {

            telemetry.addData("Starting grabber test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Open", "");
                telemetry.update();
                launcherManager.SetDirection(DriveManager.DriveDirection.FORWARD);
                launcherManager.SetPower(1.0);
                telemetry.addData("Close", "");
                telemetry.update();

                sleep(2000);
                break;
            }

        }
    }

    public void TestRamp() {

        if (opModeIsActive()) {

            telemetry.addData("Starting arm test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Move to min", "");
                telemetry.update();
                rampManager.SetDirection(DriveManager.DriveDirection.FORWARD);
                rampManager.SetPower(1.0);
                telemetry.addData("Move to max", "");
                telemetry.update();
                sleep(1000);

                telemetry.addData("Move to 0.2", "");
                telemetry.update();
                sleep(2000);

                break;
            }

        }
    }

    public void TestIntake() {

        if (opModeIsActive()) {

            telemetry.addData("Starting arm test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Move to min", "");
                telemetry.update();
                intakeManager.SetDirection(DriveManager.DriveDirection.FORWARD);
                intakeManager.SetPower(1.0);
                telemetry.addData("Move to max", "");
                telemetry.update();
                sleep(1000);

                telemetry.addData("Move to 0.2", "");
                telemetry.update();
                sleep(2000);

                break;
            }

        }
    }

    public void TestRotate180() {
         if (opModeIsActive()) {

            telemetry.addData("Starting rotate test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Starting rotate test timed", "");
                telemetry.update();
                //driveManager.TurnTimed(DRIVE_SPEED, 2000);
                //telemetry.addData("Starting rotate test encoders", "");
                //telemetry.update();
                driveManager.TurnUsingEncoders(DriveManager.DriveDirection.LEFT, DRIVE_SPEED, 10);
                sleep(100);
                break;
            }

        }
    }
}