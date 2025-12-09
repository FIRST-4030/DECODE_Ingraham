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

package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
import org.firstinspires.ftc.teamcode.SensorGoBildaPinpoint;
import org.firstinspires.ftc.teamcode.Shooter;

@Autonomous(name="Mecanum Auto Close", group="Linear OpMode")
public class MecanumAutoClose extends LinearOpMode {

    // Declare OpMode members.

    Chassis ch;
    Shooter shooterLeft;
    Shooter shooterRight;
    Servo launchFlapLeft;
    Servo launchFlapRight;
    Servo flipper;

    //private DigitalChannel redLED;
    //private DigitalChannel greenLED;

    ElapsedTime runtime = new ElapsedTime();
    private int startDelay = 0;

    public static int decimation = 3;
    public static double power = 0.7;
    double yawImu;
    YawPitchRollAngles orientation;

//    GoalTag goalTag;
    GoalTagLimelight limelight;

    private boolean shooting = false;
    private int teamID;

    // This declares the IMU needed to get the current direction the robot is facing
    //IMU imu;

    SensorGoBildaPinpoint pinpointc;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        flipper = hardwareMap.get(Servo.class, "flipper");

        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);
        shooterRight = new Shooter(hardwareMap, "shooterRight", false);

        ch = new Chassis(hardwareMap);

        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");

        launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");

//        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
//                RevHubOrientationOnRobot.LogoFacingDirection.UP;
//        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
//        RevHubOrientationOnRobot orientationOnRobot = new
//                RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));

//        goalTag = new GoalTag();
//        goalTag.init(hardwareMap);

        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap,telemetry);
        GlobalStorage.setPattern(null);
        GlobalStorage.setAlliance(-1);

//        pinpointc = new SensorGoBildaPinpoint();
//        pinpointc.initOdometry(hardwareMap, 0, 0, 0);

//        pinpoint = new GoBildaPinpointDriver();

        do {
            //goalTag.initProcessNoGoal();
            limelight.readObelisk(telemetry);
            //GlobalStorage.setPattern(goalTag.getObelisk());
            GlobalStorage.setPattern(limelight.getObelisk());


            telemetry.addData("Pattern", limelight.getObelisk());
            telemetry.addData("Is Tag Recent", limelight.seeObelisk);
            telemetry.addData("team ID", teamID);
            telemetry.addLine("Press b for red, x for blue, y adds delay, a removes delay");
            telemetry.addData("Start Delay", startDelay);
            telemetry.update();
            if (gamepad1.bWasPressed()) {
                //goalTag.targetAprilTagID = 24;
                teamID = 24;
                GlobalStorage.setAlliance(24);
            } else if (gamepad1.xWasPressed()) {
                //goalTag.targetAprilTagID = 20;
                teamID = 20;
                GlobalStorage.setAlliance(20);
            } else if (gamepad1.yWasPressed()) {
                startDelay += 2;
            } else if (gamepad1.aWasPressed()) {
                startDelay -= 1;
            }
        } while (opModeInInit());

        runtime.reset();
        //imu.resetYaw();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            launchFlapLeft.setPosition(0.3);
            launchFlapRight.setPosition(0.4);
            if (teamID == 24) {
                limelight.setTeam(24);
            } else if (teamID == 20) {
                limelight.setTeam(20);
            }
            sleep(startDelay*1000);
            if (limelight.getID() == 24) {
                ch.turn(0.5, 1300);
                //moveForward(-0.5, 1600);
                ch.moveForward(-0.5, 1200);
            } else {
                ch.turn(-0.5, 1300);
                ch.moveForward(-0.5, 1200);
            }

            fireVolleySorted();

            flipper.setPosition(0.525);
            shooterLeft.targetVelocity = 0;
            shooterRight.targetVelocity = 0;
            //move to x=11, same y
            // turn to face artifacts
            // collect artifacts

            if (teamID == 24)
            {
              // moveForward(-0.5,1000);
               //600
               ch.turn(0.5,1200);
               //800
                //1300
               ch.moveForward(0.5, 1000);
               //900
            }
            else {
                //moveForward(-0.5, 1000);
                //600
                ch.turn(-0.5, 1200);
                //pw
                //-0.5
                //ms
                //800
                //1300
                ch.moveForward(0.5, 1000);
                //900
            }
            break;

        }
    }

    public void fireVolleySorted() {
        runtime.reset();
        int velLeft = 27;
        int velRight = 28;
        //while (runtime.seconds() < 0.1) {
//            limelight.process(telemetry);
//            velLeft = (limelight.getRange() + 202.17 - 10) / 8.92124;
//            velRight = (limelight.getRange() + 202.17 - 10) / 8.92124;
//        }
        if (limelight.getObelisk().equals("PGP")) {
            fireShooterLeft(velLeft);
            fireShooterRight(velRight);
            flipper.setPosition(1);
            sleep(250);
            fireShooterLeft(velLeft);
        } else if (limelight.getObelisk().equals("GPP")) {
            fireShooterRight(velRight);
            fireShooterLeft(velLeft);
            flipper.setPosition(1);
            sleep(500);
            fireShooterLeft(velLeft);
        } else if (limelight.getObelisk().equals("PPG")) {
            fireShooterLeft(velLeft);
            sleep(250);
            flipper.setPosition(1);
            fireShooterLeft(velLeft);
            fireShooterRight(velRight);
        }
    }
    private void movePinpoint() {
        telemetry.addLine("Push your robot around to see it track");
        telemetry.addLine("Press A to reset the position");
        if(gamepad1.a){
            // You could use readings from April Tags here to give a new known position to the pinpoint
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();

        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
    }

//    private void moveForward(double power, double mseconds){
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//
//        while (timer.milliseconds() < mseconds) {
//            frontLeftDrive.setPower(power);
//            backLeftDrive.setPower(power);
//            frontRightDrive.setPower(power);
//            backRightDrive.setPower(power);
//        }
//
//        stopMotors();
//    }


//    private void rotateTo(double targetAngle) {
//        double Kp = 0.03;  // Proportional gain (tune this)
//        double Kd = 0.0;  // derivative gain
//        double minPower = 0.3;
//        double maxPower = 0.5;
//        double tolerance = 3.0; // degrees
//        double lastError = 0;
//        double derivative;
//        double currentAngle, error, turnPower;
//
//        long lastTime = System.nanoTime();
//
//        while (true) {
//            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//
//            error = -targetAngle - currentAngle;
//            error = (error + 540) % 360 - 180; // Wrap error to [-180, 180] range
//
//            long now = System.nanoTime();
//            double deltaTime = (now - lastTime) / 1e9;
//            lastTime = now;
//
//            derivative = (error - lastError) / deltaTime;
//            lastError = error;
//
//            if (Math.abs(error) < tolerance) break;
//
//            turnPower = Kp * error + Kd * derivative;
//
//            // Enforce minimum power
//            if (Math.abs(turnPower) < minPower) {
//                turnPower = Math.signum(turnPower) * minPower;
//            }
//            // Clamp maximum power
//            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));
//
//            telemetry.addData("Target (deg)", "%.2f", targetAngle);
//            telemetry.addData("Current (deg)", "%.2f", currentAngle);
//            telemetry.addData("Error", "%.2f", error);
//            telemetry.addData("Turn Power", "%.2f", turnPower);
//            telemetry.update();
//
//            frontLeftDrive.setPower(-turnPower);
//            backLeftDrive.setPower(-turnPower);
//            frontRightDrive.setPower(turnPower);
//            backRightDrive.setPower(turnPower);
//        }
//    }



    public void fireShooterLeft(double velocity) {
        shooting = true;
        shooterLeft.targetVelocity = velocity;
        ElapsedTime timer = new ElapsedTime();

        while (!shooterLeft.atSpeed()) {
            shooterLeft.overridePower();
        }
        timer.reset();
        launchFlapLeft.setPosition(0);
        while (timer.seconds() < 0.5) {
            shooterLeft.overridePower();
        }
        launchFlapLeft.setPosition(0.3);
        while (timer.seconds() < 1) {
            shooterLeft.overridePower();
        }
    }
    public void fireShooterRight(double velocity) {
        shooting = true;
        shooterRight.targetVelocity = velocity;
        ElapsedTime timer = new ElapsedTime();

        while (!shooterRight.atSpeed()) {
            shooterRight.overridePower();
        }
        timer.reset();
        launchFlapRight.setPosition(0.7);
        while (timer.seconds() < 0.5) {
            shooterRight.overridePower();
        }
        launchFlapRight.setPosition(0.4);
        while (timer.seconds() < 1) {
            shooterRight.overridePower();
        }
    }
}
