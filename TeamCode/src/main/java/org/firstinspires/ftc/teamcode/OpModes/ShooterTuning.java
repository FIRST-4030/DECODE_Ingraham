package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.GoalTag;
import org.firstinspires.ftc.teamcode.RobotTeleopMecanumFieldRelativeDriveFinleyCopy;
@Disabled
@TeleOp(name = "ShooterTuning")
public class ShooterTuning extends LinearOpMode {
    //private DcMotorEx shooter;
    private GoalTag goalTag;
    RobotTeleopMecanumFieldRelativeDriveFinleyCopy robot;
    private double targetPower = 0.3; // rotations per second (max is 60)
    public static final double NEW_P = 150.0; // default is 10.0
    public static final double NEW_I = 0; // default is 3.0
    public static final double NEW_D = 0; // default is 0.0
    public static final double NEW_F = 15.0; // default is 0.0



    public void runOpMode() {
//        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//        shooter.setDirection(DcMotor.Direction.FORWARD);
//        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooter.setPower(targetPower);

        goalTag = new GoalTag();
        robot = new RobotTeleopMecanumFieldRelativeDriveFinleyCopy(hardwareMap);

        goalTag.init(hardwareMap);

        do {
            goalTag.initProcess();
            telemetry.addData("Pattern", goalTag.getObelisk());
            telemetry.addData("team ID", goalTag.getGoalTagID());
            telemetry.update();
        } while(opModeInInit());

        //PIDFCoefficients pidfOrig = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        //shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // Re-read coefficients and verify change.
        //PIDFCoefficients pidfModified = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            goalTag.process();
            robot.loop(telemetry, gamepad1, goalTag);

            if (gamepad1.yWasPressed()) {
                targetPower += 0.05;
                //shooter.setPower(targetPower); // max speed is about 55 RPS (empirically determined)
            } else if (gamepad1.aWasPressed()) {
                targetPower -= 0.05;
                //shooter.setPower(targetPower); // max speed is about 55 RPS (empirically determined)
            }

            telemetry.addData("targetPower", targetPower);
            //telemetry.addData("ShooterPower", (shooter.getPower()));
            //telemetry.addData("ShooterVelocity", shooter.getVelocity());
            telemetry.addData("GoalRange", (goalTag.getRange()));
            telemetry.addData("GoalBearing", (goalTag.getBearing()));
            telemetry.addData("Obelisk?", goalTag.getObelisk());
            telemetry.update();
        }
    }
}