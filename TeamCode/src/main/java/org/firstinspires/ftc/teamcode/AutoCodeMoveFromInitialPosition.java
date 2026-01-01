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


@Autonomous(name="Auto Code Move From Initial Position", group="")
public class AutoCodeMoveFromInitialPosition extends LinearOpMode {
    static final double     DRIVE_SPEED             = 1;
    static final double     DRIVE_INCREASED_SPEED             = 1;
    static final double     TURN_SPEED              = 1;

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

        moveOutFromInitial();

    }

    public void moveOutFromInitial() {

        //Going backward to shoot
        if (opModeIsActive()) {

            telemetry.addData("Starting to move", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {

                telemetry.addData("go forward", "");
                telemetry.update();

                driveManager.MoveStraightToPosition(DriveManager.DriveDirection.BACKWARD, DRIVE_SPEED, 30);

                telemetry.addData("go forward", "");
                telemetry.update();

                break;
            }

        }
    }

}
