package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Pinpoint {

    private final Telemetry telemetry;
    private final Chassis chassis;

    public GoBildaPinpointDriver odo;

    Datalog datalog;
    boolean logData;

    private double kP = 0.01, kI = 0.0, kD = 0.0001;
    private double xkP = 0.015, xkI = 0.0, xkD = 0.000;
    private double ykP = 0.04, ykI = 0.0, ykD = 0.000;
    double kP_turn = 0.3;
    double maxSpeed = 0.6; // Maximum drive speed
    double minSpeed = 0.1; // Maximum drive speed
    double tolerance = 0.5; // Position tolerance in inches
    double maxPower;

    double currentX, currentY, currentHeading;
    double targetX, targetY;
    double errorX, errorY, distanceToTarget;
    double vX, vY, speed, uncorrectedSpeed;
    double strafeSpeed, forwardSpeed;
    double headingError, uncorrectedHeading;
    Pose2D pose;

    public Pinpoint(HardwareMap hw, Chassis ch, Telemetry tele, boolean log) {

        chassis = ch;
        telemetry = tele;
        logData = log;

        odo = hw.get(GoBildaPinpointDriver.class,"odo");
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-110, 100, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);
        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
           odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        odo.update();
        odo.resetPosAndIMU();

        // Initialize the datalog
        if (logData) {
            datalog = new Datalog("Pinpoint");
        }
    }

    public void setEncoderDirection(GoBildaPinpointDriver.EncoderDirection xEncoder,
                                    GoBildaPinpointDriver.EncoderDirection yEncoder) {
        odo.setEncoderDirections(xEncoder, yEncoder);
    }

    @SuppressLint("DefaultLocale")
    public void moveTo(double targetXa, double targetYa, double newHeading) {

        targetX = targetXa;
        targetY = targetYa;
        double targetHeading = Math.toRadians(newHeading);

        long lastTime = System.nanoTime();
        double lastErrorX = 0, lastErrorY = 0;
        double derivativeX, derivativeY, integralX = 0, integralY = 0;
        double headingTolerance = Math.toRadians(5); // 5 degrees

        while (true) {

            odo.update();

            pose = odo.getPosition();

            // Get current position from Pinpoint
            currentX = pose.getX(DistanceUnit.INCH);
            currentY = pose.getY(DistanceUnit.INCH);

            // Calculate error
            errorX = targetX - currentX;
            errorY = targetY - currentY;
            errorY = 0;

            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;
            lastTime = now;

            derivativeX = (errorX - lastErrorX) / deltaTime;
            derivativeY = (errorY - lastErrorY) / deltaTime;

            integralX += errorX * deltaTime;
            integralY += errorY * deltaTime;

            lastErrorX = errorX;
            lastErrorY = errorY;

//            if (Math.abs(errorX) < tolerance) break;

            // Calculate distance to target
            distanceToTarget = Math.hypot(errorX, errorY);

//            if (distanceToTarget < tolerance && Math.abs(headingError) < headingTolerance) break;
            if (distanceToTarget < tolerance) break;

            // Calculate velocities using proportional control
//            vX = errorX * kP + kI * integralX + kD * derivativeX;
            vX = errorX * xkP + kI * integralX + kD * derivativeX;
            vY = errorY * ykP + kI * integralY + kD * derivativeY;

            // Limit to max speed
            speed = Math.hypot(vX, vY);
            uncorrectedSpeed = speed;
            if (speed > maxSpeed) {
                vX = (vX / speed) * maxSpeed;
                vY = (vY / speed) * maxSpeed;
            } else if (speed > 0 && speed < minSpeed && distanceToTarget > tolerance) {
                // Scale up if too slow (but not at target)
                vX = vX / speed * minSpeed;
                vY = vY / speed * minSpeed;
            }

            // Calculate heading error
            currentHeading = pose.getHeading(AngleUnit.RADIANS);
            headingError = angleWrap(targetHeading - currentHeading);
            uncorrectedHeading = headingError;

            // Wrap angle to [-π, π]
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;
            double turnSpeed = Range.clip(headingError * 0.3, -1.0, 1.0);

            // Drive field-relative (no rotation for this example)
//            driveFieldRelative(vX, vY, turnSpeed);

            // Calculate drive speeds
            forwardSpeed = Range.clip(vX, -1.0, 1.0);
            strafeSpeed  = Range.clip(vY, -1.0, 1.0);

            // Calculate turn speed (only turn aggressively when close to target)
//            if (distanceToTarget < 5.0) {  // Within 12 inches
//                turnSpeed = Range.clip(headingError * kP_turn, -1.0, 1.0);
//            } else {
//                turnSpeed = 0;  // Don't rotate until close
//            }

            // Drive the robot
            driveFieldRelative(strafeSpeed, forwardSpeed, turnSpeed);

//            telemetry.addLine(String.format("Distance to Target: %6.2f",distanceToTarget));
//            telemetry.addLine(String.format("vX: %6.2f, vY: %6.2f",vX,vY));
//            telemetry.update();

            if (logData) logOneSample();
        }

        driveFieldRelative(0,0,0);
//        chassis.stopMotors();
    }
    /**
     * Wrap angle to [-PI, PI]
     */
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double getKp() {
        return this.kP;
    }

    public double getKi() {
        return this.kI;
    }

    public double getKd() {
        return this.kD;
    }

    public double getMaxSpeed() {
        return this.maxSpeed;
    }

    public double getTolerance() {
        return this.tolerance;
    }

    public void setKp(double kP) {
        this.kP = kP;
    }

    public void setKi(double kI) {
        this.kI = kI;
    }

    public void setKd(double kD) {
        this.kD = kD;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Field-relative drive using mecanum wheels
     */
    public void driveFieldRelative(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        // Get current heading
        Pose2D pose = odo.getPosition();
        double heading = pose.getHeading(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the robot's rotation
        double rotatedStrafe = strafeSpeed * Math.cos(-heading) - forwardSpeed * Math.sin(-heading);
        double rotatedForward = strafeSpeed * Math.sin(-heading) + forwardSpeed * Math.cos(-heading);

        // Calculate wheel powers
        double frontLeftPower = rotatedForward + rotatedStrafe + turnSpeed;
        double frontRightPower = rotatedForward - rotatedStrafe - turnSpeed;
        double backLeftPower = rotatedForward - rotatedStrafe + turnSpeed;
        double backRightPower = rotatedForward + rotatedStrafe - turnSpeed;

        // Normalize if any power exceeds 1.0
        maxPower = Math.max(Math.abs(frontLeftPower),
                          Math.max(Math.abs(frontRightPower),
                          Math.max(Math.abs(backLeftPower),
                          Math.abs(backRightPower))));

//        maxPower = maxSpeed;
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        chassis.frontLeftDrive.setPower(frontLeftPower);
        chassis.frontRightDrive.setPower(frontRightPower);
        chassis.backLeftDrive.setPower(backLeftPower);
        chassis.backRightDrive.setPower(backRightPower);
    }

    private void logOneSample() {
        datalog.maxPower.set(maxPower);
        datalog.targetX.set(targetX);
        datalog.vX.set(vX);
        datalog.vY.set(vY);
        datalog.speed.set(speed);
        datalog.uncorrectedSpeed.set(uncorrectedSpeed);
        datalog.targetY.set(targetY);
        datalog.currentX.set(currentX);
        datalog.currentY.set(currentY);
        datalog.currentHeading.set(Math.toDegrees(currentHeading));
        datalog.uncorrectedHeading.set(Math.toDegrees(uncorrectedHeading));
        datalog.errorX.set(errorX);
        datalog.errorY.set(errorY);
        datalog.distanceToTarget.set(distanceToTarget);
        datalog.writeLine();
    }

    /**
     * Datalog class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog {
        /*
         * The underlying datalogger object - it cares only about an array of loggable fields
         */
        private final Datalogger datalogger;
        /*
         * These are all of the fields that we want in the datalog.
         * Note: Order here is NOT important. The order is important
         *       in the setFields() call below
         */
        public Datalogger.GenericField maxPower = new Datalogger.GenericField("MaxPower");
        public Datalogger.GenericField targetX = new Datalogger.GenericField("Target-X");
        public Datalogger.GenericField vX = new Datalogger.GenericField("vX");
        public Datalogger.GenericField vY = new Datalogger.GenericField("vY");
        public Datalogger.GenericField speed = new Datalogger.GenericField("speed");
        public Datalogger.GenericField uncorrectedSpeed = new Datalogger.GenericField("Uncorr speed");
        public Datalogger.GenericField targetY = new Datalogger.GenericField("Target-Y");
        public Datalogger.GenericField currentX = new Datalogger.GenericField("X");
        public Datalogger.GenericField currentY = new Datalogger.GenericField("Y");
        public Datalogger.GenericField currentHeading = new Datalogger.GenericField("Heading");
        public Datalogger.GenericField uncorrectedHeading = new Datalogger.GenericField("Uncorr Heading");
        public Datalogger.GenericField errorX = new Datalogger.GenericField("Err X");
        public Datalogger.GenericField errorY = new Datalogger.GenericField("Err Y");
        public Datalogger.GenericField distanceToTarget = new Datalogger.GenericField("Delta");

        public Datalog(String name) {
            datalogger = new Datalogger.Builder()
                .setFilename(name)
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                /*
                 * Tell it about the fields we care to log.
                 * Note: Order *IS* important here! The order in which we list the
                 *       fields is the order in which they will appear in the log.
                 */
                .setFields(
                        maxPower,
                        targetX,
                        targetY,
                        distanceToTarget,
                        vX,
                        vY,
                        speed,
                        uncorrectedSpeed,
                        currentX,
                        currentY,
                        currentHeading,
                        uncorrectedHeading,
                        errorX,
                        errorY
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
