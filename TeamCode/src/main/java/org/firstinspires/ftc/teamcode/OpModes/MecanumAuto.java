package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class MecanumAuto extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime    = new ElapsedTime();
    private DcMotor     leftFront  = null;
    private DcMotor     rightFront = null;

    private int allianceSide;

    @Override
    public void RunOpMode() {
        /*
         * Code that runs ONCE when the driver hits INIT
         *    Tasks to include are,
         *       - Initialize all motors, sensors, etc. via calls to "hardwareMap"
         *       - Initialize all class level variables
         */
        // Initialize the hardware variables
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();




    }
}