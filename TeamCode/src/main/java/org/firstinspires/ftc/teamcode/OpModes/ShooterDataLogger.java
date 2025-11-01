package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.OpModes.PIDF_Example.COUNTS_PER_REV;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.GoalTag;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Disabled
@TeleOp(name = "ShooterDataLogger")
public class ShooterDataLogger extends LinearOpMode{

    private DcMotorEx shooter;
    private Servo launchFlap;
    private GoalTag goalTag;
    private double initPos = 0.5;

    private double goalRange;
    private double goalBearing;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    Datalog AimTestDatalog; // create the data logger object
    private double targetPower = 0;

    private boolean goal;

    private int i = 0; // loop counter
    private int j = 0;

    private int k = 0;

    private boolean readyToShoot = false;
    public static final double NEW_P = 150.0; // default is 10.0
    public static final double NEW_I = 0; // default is 3.0
    public static final double NEW_D = 0; // default is 0.0
    public static final double NEW_F = 15.0; // default is 0.0


    @Override
    public void runOpMode() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        launchFlap = hardwareMap.get(Servo.class, "launchFlap");
        launchFlap.setDirection(Servo.Direction.FORWARD);

        goalTag = new GoalTag();

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Initialize the datalog
        AimTestDatalog = new Datalog("launch log");
        // wait for start command

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        goalTag.init(hardwareMap);

        do {
            goalTag.initProcess();
            telemetry.addData("Pattern", goalTag.getObelisk());
            telemetry.addData("team ID", goalTag.getGoalTagID());
            telemetry.update();
        } while(opModeInInit());

        // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
        PIDFCoefficients pidfOrig = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // Re-read coefficients and verify change.
        PIDFCoefficients pidfModified = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Not sure if setVelocity is working properly
        // angular rate in counts (ticks) per second
        // max speed is about 55 RPS (empirically determined)


        // display info to user
        while (opModeIsActive()) {
            i++;
            k++;
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
            //shooter.setVelocity(targetVelocity*COUNTS_PER_REV);
            // setPower is required, in addition to setVelocity
            // targetPower = (goalRange+241.28)/10.473;

            goalTag.process();
            goalRange = goalTag.getRange();
            goalBearing = goalTag.getBearing();
            targetPower = (goalRange+319.94289)/670.11831;
            shooter.setPower(targetPower);


            if (gamepad1.leftBumperWasPressed()) {
                goal = true;
                telemetry.addData("goal",goal);
                telemetry.addData("targetVelocity", targetPower);
                telemetry.update();
                AimTestDatalog.goalBool.set(goal);
                AimTestDatalog.targetPower.set(targetPower);
                AimTestDatalog.goalRange.set(goalRange);
                AimTestDatalog.goalBearing.set(goalBearing);
                AimTestDatalog.writeLine();
            } else if (gamepad1.rightBumperWasPressed()) {
                goal = false;
                telemetry.addData("goal", goal);
                telemetry.addData("targetVelocity", targetPower);
                telemetry.update();
                AimTestDatalog.goalBool.set(goal);
                AimTestDatalog.targetPower.set(targetPower);
                AimTestDatalog.goalRange.set(goalRange);
                AimTestDatalog.goalBearing.set(goalBearing);
                AimTestDatalog.writeLine();
            } else if (gamepad1.yWasPressed()) {
                targetPower += 0.005;
                shooter.setPower(targetPower); // max speed is about 55 RPS (empirically determined)
            } else if (gamepad1.aWasPressed()) {
                targetPower -= 0.005;
                shooter.setPower(targetPower); // max speed is about 55 RPS (empirically determined)
            } else if (gamepad1.right_trigger == 1) {
                launchFlap.setPosition(0);
                i = 0;
                readyToShoot = false;
            }
            if (i > 500) {
               launchFlap.setPosition(initPos);
               i = 0;
            }
//            if (readyToShoot) {
//                if (j < 1) {
//                    launchFlap.setPosition(0.3);
//                    k = 0;
//                    j++;
//                }
//                if (k > 200) {
//                    launchFlap.setPosition(0);
//                    j--;
//                }

            
            telemetry.addData("i", i);
            //telemetry.addData("j", j);
            //telemetry.addData("k", k);

            telemetry.addData("val", gamepad1.right_trigger);


            telemetry.addData("targetPower", targetPower);
            telemetry.addData("currentPower", shooter.getPower());
            telemetry.addData("GoalRange", (goalRange));
            telemetry.addData("GoalBearing", (goalBearing));
            telemetry.update();

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
        public Datalogger.GenericField shooterPower = new Datalogger.GenericField("shooterVelocity");
        public Datalogger.GenericField targetPower = new Datalogger.GenericField("targetVelocity");
        public Datalogger.GenericField goalBool = new Datalogger.GenericField("goalBool");
        public Datalogger.GenericField goalRange = new Datalogger.GenericField("goalRange");
        public Datalogger.GenericField goalBearing = new Datalogger.GenericField("goalBearing");

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
                            goalBool,
                            targetPower,
                            goalRange,
                            goalBearing
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


