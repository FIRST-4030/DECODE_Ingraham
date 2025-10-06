package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PIDF_Example.COUNTS_PER_REV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "ShooterDataLogger")
public class ShooterDataLogger extends LinearOpMode{

    private DcMotorEx shooter;
    private Servo launchFlap;
    private GoalTag goalTag;

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
    private double targetVelocity = 30; // rotations per second (max is 60)

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
        launchFlap.setPosition(1);
        launchFlap.setDirection(Servo.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Initialize the datalog
        AimTestDatalog = new Datalog("launch log");
        // wait for start command
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();



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
            targetVelocity = (goalRange+241.28)/10.473;
            shooter.setPower(targetVelocity/55);
            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();
            if (gamepad1.leftBumperWasPressed()) {
                goal = true;
                telemetry.addData("goal",goal);
                telemetry.addData("targetVelocity", targetVelocity);
                telemetry.update();
                AimTestDatalog.goalBool.set(goal);
                AimTestDatalog.targetVelocity.set(targetVelocity);
                AimTestDatalog.goalRange.set(goalRange);
                AimTestDatalog.goalBearing.set(goalBearing);
                AimTestDatalog.writeLine();
            } else if (gamepad1.rightBumperWasPressed()) {
                goal = false;
                telemetry.addData("goal", goal);
                telemetry.addData("targetVelocity", targetVelocity);
                telemetry.update();
                AimTestDatalog.goalBool.set(goal);
                AimTestDatalog.targetVelocity.set(targetVelocity);
                AimTestDatalog.goalRange.set(goalRange);
                AimTestDatalog.goalBearing.set(goalBearing);
                AimTestDatalog.writeLine();
            } else if (gamepad1.yWasPressed()) {
                targetVelocity += 0.25;
                shooter.setVelocity(targetVelocity*COUNTS_PER_REV);
                shooter.setPower(targetVelocity/55); // max speed is about 55 RPS (empirically determined)
            } else if (gamepad1.aWasPressed()) {
                targetVelocity -= 0.25;
                shooter.setVelocity(targetVelocity*COUNTS_PER_REV);
                shooter.setPower(targetVelocity/55); // max speed is about 55 RPS (empirically determined)
            } else if (gamepad1.right_trigger == 1) {
                launchFlap.setPosition(0.7);
                i = 0;
                readyToShoot = false;
            }
            if (i > 500) {
                launchFlap.setPosition(1);
                i = 0;
            }
            if (readyToShoot) {
                if (j < 1) {
                    launchFlap.setPosition(0.3);
                    k = 0;
                    j++;
                }
                if (k > 200) {
                    launchFlap.setPosition(0);
                    j--;
                }

            }
            telemetry.addData("i", i);
            telemetry.addData("j", j);
            telemetry.addData("k", k);

            telemetry.addData("val", gamepad1.right_trigger);


            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.addData("currentVelocity", shooter.getVelocity());
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
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                .setDrawCubeProjection(true) // defaults to false
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                goalRange = detection.ftcPose.range;
                goalBearing = detection.ftcPose.bearing;

            }
        }   // end for() loop

        // Add "key" information to telemetry


    }   // end method telemetryAprilTag()


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
                            targetVelocity,
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


