package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.PIDF_Example.COUNTS_PER_REV;

import static java.lang.Runtime.getRuntime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp(name = "ShooterDataLogger")
public class ShooterDataLogger extends LinearOpMode{

    DcMotorEx shooter;
    PIDF_Example.Datalog datalog; // create the data logger object
    private double targetVelocity = 30; // rotations per second (max is 60)

    private boolean goal;

    private int i = 0; // loop counter

    public void runOpMode() {
        shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // wait for start command
        waitForStart();

        // display info to user
        while (opModeIsActive()) {
            /*
            i++;
            double currentVelocity = shooter.getVelocity(AngleUnit.DEGREES) / COUNTS_PER_REV;
            telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                    pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
            telemetry.addData("shooterVelocity", currentVelocity);
            telemetry.update();
            */
            if (gamepad1.left_bumper)
            {
                goal = true;
                datalog.targetVelocity.set(targetVelocity);
                datalog.writeLine();
            }
            if (gamepad1.right_bumper)
            {
                goal = false;
                datalog.targetVelocity.set(targetVelocity);
                datalog.writeLine();
            }



            // Data log
            // Note that the order in which we set datalog fields
            // does *not* matter! Order is configured inside the Datalog class constructor.
            //datalog.targetVelocity.set(targetVelocity);
            //datalog.writeLine();
        }
    }


    /**
     * Datalog class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField loopCounter = new Datalogger.GenericField("LoopCounter");
        public Datalogger.GenericField runTime = new Datalogger.GenericField("RunTime");
        public Datalogger.GenericField deltaTime = new Datalogger.GenericField("deltaTime");
        public Datalogger.GenericField shooterVelocity = new Datalogger.GenericField("shooterVelocity");
        public Datalogger.GenericField targetVelocity = new Datalogger.GenericField("targetVelocity");
        public Datalogger.GenericField goal = new Datalogger.GenericField("goal");

        public Datalog(String name) {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            //loopCounter,
                            //runTime,
                            //deltaTime,
                            //shooterVelocity,
                            goal,
                            targetVelocity
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.

        public void writeLine() {
            datalogger.writeLine();
        }
    }
}


