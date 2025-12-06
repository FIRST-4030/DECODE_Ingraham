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


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
import org.firstinspires.ftc.teamcode.SensorGoBildaPinpoint;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Pinpoint;

@Autonomous(name="Mecanum Auto Close Pinpoint", group="Linear OpMode")
public class MecanumAutoClosePinpoint extends LinearOpMode {

    // Declare OpMode members.
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    Shooter shooterLeft;
    Shooter shooterRight;
    Servo launchFlapLeft;
    Servo launchFlapRight;
    Servo flipper;
    Pinpoint pinpoint;
    Chassis ch;
    private double currentX;
    private double currentY;
    private double currentAngle;

    private double velLeft = 0;
    private double velRight = 0;

    private boolean testingMode = false;

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



    @Override
    public void runOpMode() {
        pinpoint = new Pinpoint(hardwareMap,ch,telemetry,false);

        flipper = hardwareMap.get(Servo.class, "flipper");

        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);
        shooterRight = new Shooter(hardwareMap, "shooterRight", false);

        ch = new Chassis(hardwareMap);

        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");

        launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");

        //imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        //imu.initialize(new IMU.Parameters(orientationOnRobot));

//        goalTag = new GoalTag();
//        goalTag.init(hardwareMap);
        ch = new Chassis(hardwareMap);

        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap,telemetry);
        GlobalStorage.setPattern(null);
        GlobalStorage.setAlliance(-1);

        pinpoint.setEncoderDirection(org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,0));

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
            if (teamID == 24) {
                limelight.setTeam(24);
            } else if (teamID == 20) {
                limelight.setTeam(20);
            }
            sleep(startDelay*1000);
            if (limelight.getID() == 24) {
                //turn(0.5, 1300);
                turnTo(-135, 0.5);
                //moveForward(-0.5, 1200);
                moveX(48, -0.5);

            } else {
                //turn(-0.5, 1300);
                turnTo(135, 0.5);
                //moveForward(-0.5, 1200);
                moveX(48, -0.5);
            }

            if (limelight.getObelisk().equals("PGP")) {
                fireShooterLeft(27);
                fireShooterRight(28);
                flipper.setPosition(1);
                sleep(100);
                flipper.setPosition(0);
                fireShooterLeft(27);
            } else if (limelight.getObelisk().equals("GPP")) {
                fireShooterRight(28);
                fireShooterLeft(27);
                flipper.setPosition(1);
                sleep(100);
                flipper.setPosition(0);
                fireShooterLeft(27);
            } else if (limelight.getObelisk().equals("PPG")) {
                fireShooterLeft(27);
                sleep(100);
                flipper.setPosition(1);
                sleep(100);
                flipper.setPosition(0);
                fireShooterLeft(27);
                fireShooterRight( 28);
            } else {
                fireShooterLeft(27);
                fireShooterRight(28);
            }
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
               //turn(0.5,1200);
               turnTo(-135,0.5);
               //800
                //1300
               //moveForward(0.5, 1000);
               moveX(48,0.5);
               //900
            }
            else {
                //moveForward(-0.5, 1000);
                //600
                //turn(-0.5, 1200);
                turnTo(135,0.5);
                //pw
                //-0.5
                //ms
                //800
                //1300
                //moveForward(0.5, 1000);
                moveX(48,0.5);
                //900
            }
            break;

        }
    }

//    private void movePinpoint() {
//        telemetry.addLine("Push your robot around to see it track");
//        telemetry.addLine("Press A to reset the position");
//        if(gamepad1.a){
//            // You could use readings from April Tags here to give a new known position to the pinpoint
//            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
//        }
//        pinpoint.update();
//        Pose2D pose2D = pinpoint.getPosition();
//
//        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
//        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
//        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
//    }


    public void fireVolleySorted() {
        if (limelight.getObelisk().equals("PGP") && !testingMode) {
            fireShooterLeft(velLeft);
            fireShooterRight(velRight);
            flipper.setPosition(1);
            sleep(100);
            fireShooterLeft(velLeft);
        } else if (limelight.getObelisk().equals("GPP") && !testingMode) {
            fireShooterRight(velRight);
            fireShooterLeft(velLeft);
            flipper.setPosition(1);
            sleep(100);
            fireShooterLeft(velLeft);
        } else if (limelight.getObelisk().equals("PPG") && !testingMode) {
            fireShooterLeft(velLeft);
            sleep(100);
            flipper.setPosition(1);
            sleep(100);
            fireShooterLeft(velLeft);
            fireShooterRight(velRight);
        }
    }
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
    public void turnToAprilTagLimelight() {
        if (limelight.getRange() < 100) {
            turnTo(0.25, 0.5);
        } else {
            if (limelight.getID() == 20) {
                turnTo(0.25, 2);
            } else if (limelight.getID() == 24) {
                turnTo(0.25, -2);
            }
        }
    }
    private void turnTo(double t_angle, double power) {
        while (true) {
            pinpoint.odo.update();
            Pose2D pose = pinpoint.odo.getPosition();
            currentAngle = pose.getHeading(AngleUnit.DEGREES);

            //double error = t_angle+currentAngle;

            telemetry.addData("current angle",currentAngle);
            telemetry.addData("target angle", t_angle);
            //telemetry.addData("error", error);
            telemetry.update();
            if (Math.abs(t_angle-currentAngle) < 0.5) {
                ch.stopMotors();
                break;
            }
            if (t_angle < 0) {
                ch.moveAllMotors(power,-power,power,-power);
            } else {
                ch.moveAllMotors(-power,power,-power,power);
            }


        }
    }
    private void moveX (double inches, double power) {
        while (true) {
            pinpoint.odo.update();
            Pose2D pose = pinpoint.odo.getPosition();
            currentX = pose.getX(DistanceUnit.INCH);

            double error = inches-currentX;

            telemetry.addData("x",currentX);
            telemetry.addData("target inches",inches);
            telemetry.addData("error", error);
            telemetry.update();
            if (Math.abs(error) < 0.25) {
                ch.stopMotors();
                break;
            }
            ch.moveAllMotors(power,power,power,power);

        }
    }
    private void moveY (double inches, double power) {
        while (true) {
            pinpoint.odo.update();
            Pose2D pose = pinpoint.odo.getPosition();
            currentY = pose.getY(DistanceUnit.INCH);

            double error = inches-currentY;

            telemetry.addData("y",currentY);
            telemetry.addData("target inches",inches);
            telemetry.addData("error", error);
            telemetry.update();
            if (Math.abs(error) < 0.25) {
                ch.stopMotors();
                break;
            }
            ch.moveAllMotors(power,power,power,power);

        }
    }
}
