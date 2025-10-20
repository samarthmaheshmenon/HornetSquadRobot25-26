/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareMapper {

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public static void MapToHardware(LinearOpMode opMode, HornetRobo Robo)    {
        opMode.telemetry.addData("Hardware Mapper - MapToHardware:", "Starting to map");
        opMode.telemetry.update();

        //Mapping Wheel motors
        Robo.RightFrontMotor = opMode.hardwareMap.get(DcMotor.class, "motorRF");
        Robo.RightBackMotor = opMode.hardwareMap.get(DcMotor.class, "motorRB");
        Robo.LeftFrontMotor = opMode.hardwareMap.get(DcMotor.class, "motorLF");
        Robo.LeftBackMotor = opMode.hardwareMap.get(DcMotor.class, "motorLB");

        //Mapping Launcher Motors
        Robo.LeftLauncher = opMode.hardwareMap.get(DcMotor.class, "motorleftLauncher");
        Robo.RightLauncher = opMode.hardwareMap.get(DcMotor.class, "motorRightLauncher");

        //Mapping Ramp Motor
        Robo.RampMotor = opMode.hardwareMap.get(DcMotor.class, "motorRamp");

        //Mapping Intake Motor
        Robo.IntakeMotor = opMode.hardwareMap.get(DcMotor.class, "motorIntake");

        opMode.telemetry.addData("Hardware Mapper - MapToHardware:", "Finished map");
        opMode.telemetry.update();
    }
}



