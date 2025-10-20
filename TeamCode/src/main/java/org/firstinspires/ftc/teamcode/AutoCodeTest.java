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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Auto Code Test", group="")
public class AutoCodeTest extends LinearOpMode {
    static final double     DRIVE_SPEED             = 0.3;
    static final double     DRIVE_INCREASED_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.2;

    private HornetRobo hornetRobo;

    //manager classes
    private DriveManager driveManager;
    private ArmManager armManager;
    private GrabberManager armGrabberManager;
    private ViperSlideManager viperSlideManager;

    public void initialize()
    {
        hornetRobo = new HornetRobo();
        HardwareMapper.MapToHardware(this, hornetRobo);
        driveManager = new DriveManager(this, hornetRobo);
        viperSlideManager = new ViperSlideManager(this, hornetRobo);
        armManager = new ArmManager(this, hornetRobo);
        armGrabberManager = new GrabberManager(this, hornetRobo);
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
        TestViperSlideArmGrabber();

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
    public void TestGrabberOpenAndClose() {
        GrabberManager grabberManager = new GrabberManager(this, hornetRobo);

        if (opModeIsActive()) {

            telemetry.addData("Starting grabber test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Open", "");
                telemetry.update();
                grabberManager.OpenOrCloseGrabber(true);
                sleep(2000);
                telemetry.addData("Close", "");
                telemetry.update();

                grabberManager.OpenOrCloseGrabber(false);
                sleep(2000);
                break;
            }

        }
    }

    public void TestArm() {
        ArmManager armManager = new ArmManager(this, hornetRobo);

        if (opModeIsActive()) {

            telemetry.addData("Starting arm test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Move to min", "");
                telemetry.update();
                armManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
                armManager.MoveArmToPosition(0.2);
                telemetry.addData("Move to max", "");
                telemetry.update();
                sleep(1000);

                armManager.MoveArmToPosition(0.3);
                telemetry.addData("Move to min", "");
                telemetry.update();
                sleep(1000);

                armManager.MoveArmToPosition(0.4);
                telemetry.addData("Move to min", "");
                telemetry.update();
                sleep(1000);
                armManager.MoveArmToPosition(0.5);
                telemetry.addData("Move to min", "");
                telemetry.update();
                sleep(1000);
                armManager.MoveArmToPosition(0.6);
                telemetry.addData("Move to min", "");
                telemetry.update();
                sleep(1000);
                //sleep(1000);
               /* armManager.MoveArmToPosition(0.45);
                sleep(500);
                armManager.MoveArmToPosition(0.9);
                sleep(500);
                */
                telemetry.addData("Move to 0.2", "");
                telemetry.update();
                sleep(2000);

                break;
            }

        }
    }

    public void TestViperSlideArmGrabber() {
        ViperSlideManager vsManager = new ViperSlideManager(this, hornetRobo);
        ArmManager armManager = new ArmManager(this, hornetRobo);
        GrabberManager grabberManager = new GrabberManager(this, hornetRobo);
        vsManager.SetDirection(DriveManager.DriveDirection.FORWARD);
        if (opModeIsActive()) {

            telemetry.addData("Starting grabber test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Move forward for 1 sec", "");
                telemetry.update();
                armManager.SetDirection(DriveManager.DriveDirection.FORWARD);
                armManager.MoveArmToPosition(0.5);
                vsManager.ResetAndSetToEncoder();
                vsManager.SetDirection(DriveManager.DriveDirection.FORWARD);
                vsManager.SetPower(0.5);
                sleep(500);
                armManager.MoveArmToPosition(0.4);
                sleep(500);
                grabberManager.OpenOrCloseGrabber(true);
                sleep(500);
                grabberManager.OpenOrCloseGrabber(false);
                sleep(500);
                armManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
                armManager.MoveArmToPosition(0.5);
                vsManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
                vsManager.SetPower(0.5);
                sleep(100);
                //vsManager.SetDirection(AutoDriveManager.DriveDirection.BACKWARD);
                //vsManager.SetDirection(AutoDriveManager.DriveDirection.BACKWARD);
                sleep(3000);
                telemetry.addData("VS test done", "");
                telemetry.update();

                break;
            }

        }
    }

    public void TestViperSlideWithBrake() {
        ViperSlideManager vsManager = new ViperSlideManager(this, hornetRobo);
        vsManager.SetDirection(DriveManager.DriveDirection.FORWARD);
        if (opModeIsActive()) {

            telemetry.addData("Starting grabber test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Move forward for 1 sec", "");
                telemetry.update();
                armManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
                vsManager.ResetAndSetToEncoder();
                armManager.MoveArmToPosition(0.3);
                armGrabberManager.OpenOrCloseGrabber(false);
                //armManager.MoveArmToPosition(0.4);
                vsManager.SetDirection(DriveManager.DriveDirection.FORWARD);
                //vsManager.SetTargetPosition(20);
                vsManager.SetPower(0.3);
                sleep(800);
                //vsManager.BrakeOrReleaseViperSlide(true);
                vsManager.SetPower(0.0);
                armManager.MoveArmToPosition(0.6);
                sleep(3000);
                armGrabberManager.OpenOrCloseGrabber(false);
                vsManager.SetDirection(DriveManager.DriveDirection.BACKWARD);
                //vsManager.SetTargetPosition(20);
                vsManager.SetPower(0.5);
                sleep(200);
                vsManager.SetPower(0.0);
                armManager.MoveArmToPosition(0.3);
                sleep(2000);
                /*
                telemetry.addData("reached position after 1 sec", "");
                telemetry.update();
                vsManager.SetPower(0.0);
                sleep(1000);
                telemetry.addData("in brake mode", "");
                telemetry.update();
                armManager.MoveArmToPosition(0.5);
                sleep(3000);
                armManager.MoveArmToPosition(0.3);
                vsManager.BrakeOrReleaseViperSlide(false);
                vsManager.SetDirection(AutoDriveManager.DriveDirection.BACKWARD);
                vsManager.SetPower(0.2);
                sleep(1000);
                //vsManager.BrakeOrReleaseViperSlide(false);
                //vsManager.SetDirection(AutoDriveManager.DriveDirection.BACKWARD);
                //vsManager.SetPower(0.1);
                //sleep(500);

                 */
                //
                break;
            }

        }
    }

    public void TestRotate180() {
        // vsManager = new AutoViperSlideManager(this, hornetRobo);

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

    public void TestViperSlideUpOnButtonClick()
    {
        // vsManager = new AutoViperSlideManager(this, hornetRobo);

        if (opModeIsActive()) {

            telemetry.addData("Starting rotate test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                if (gamepad1.y){
                    armManager.MoveArmToPosition(0.3);
                    viperSlideManager.ResetAndSetToEncoder();
                    viperSlideManager.MoveStraightToPosition(DriveManager.DriveDirection.BACKWARD, 0.5,12);
                    //armManager.MoveArmToPosition(0.3);
                    //armGrabberManager.OpenOrCloseGrabber(true);
                    sleep(2000);
                    //armManager.MoveArmToPosition(0.7);
                   // armGrabberManager.OpenOrCloseGrabber(false);
                    //sleep(100);
                    break;

                }
            }

        }
    }
}