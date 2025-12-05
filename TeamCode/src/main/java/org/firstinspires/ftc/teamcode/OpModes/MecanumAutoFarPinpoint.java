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

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.Pinpoint;
import org.firstinspires.ftc.teamcode.Shooter;

@Autonomous(name="Mecanum Auto Far Pinpoint", group="Linear OpMode")
public class MecanumAutoFarPinpoint extends LinearOpMode {

    // Declare OpMode members.

    Shooter shooterLeft;
    Shooter shooterRight;
    Servo launchFlapLeft;
    Servo launchFlapRight;
    Shooter collectorBack;
    Shooter collectorFront;
    Servo flipper;
    Chassis ch;
    Pinpoint pinpoint;
    private double velLeft = 35;
    private double velRight = 34;

    private double currentX;
    private double currentAngle;
    ElapsedTime runtime = new ElapsedTime();
    public static int decimation = 3;
    public static double power = 0.7;
    private double lastError = 0;
    private double kP = 0.14;
    double yawImu;

    YawPitchRollAngles orientation;

    //GoalTag goalTag;
    GoalTagLimelight limelight;
    private int startDelay = 0;
    private int teamID;
    private boolean testingMode = false;

    private boolean shooting = false;
    private double collectorPower = 0.5;
    private static double turnOffset = 3.8;

    // This declares the IMU needed to get the current direction the robot is facing
    //IMU imu;

//    SensorGoBildaPinpoint pinpointc;
//    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        ch = new Chassis(hardwareMap);

        pinpoint = new Pinpoint(hardwareMap,ch,telemetry,false);

        collectorFront = new Shooter(hardwareMap,"collectorFront", false);

        collectorBack = new Shooter(hardwareMap,"collectorBack", false);

        flipper = hardwareMap.get(Servo.class, "flipper");

        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);
        shooterRight = new Shooter(hardwareMap, "shooterRight", false);

        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");

        launchFlapRight= hardwareMap.get(Servo.class, "launchFlapRight");

        pinpoint.setEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        Pose2D pose = pinpoint.odo.getPosition();

        pinpoint.odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,0));
        // One calibration does not necessarily clear the hardware
//        int i = 0;
//        while (i < 10) {
//            i++;

        pinpoint.odo.recalibrateIMU();
        pinpoint.odo.resetPosAndIMU();
        pinpoint.odo.update();

        telemetry.addData("Heading Scalar", pinpoint.odo.getYawScalar());
        telemetry.addData("Initial X", "%.2f", pinpoint.odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("Initial Y", "%.2f", pinpoint.odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("Initial Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Status", pinpoint.odo.getDeviceStatus());
        telemetry.update();
        //}
//        imu = hardwareMap.get(IMU.class, "imu");
//        // This needs to be changed to match the orientation on your robot
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


        do {
            limelight.readObelisk(telemetry);
            //GlobalStorage.setPattern(goalTag.getObelisk());
            GlobalStorage.setPattern(limelight.getObelisk());

            telemetry.addData("Pattern", limelight.getObelisk());
            telemetry.addData("Is Tag Recent", limelight.seeObelisk);
            telemetry.addData("team ID", teamID);
            telemetry.addData("Testing Mode", testingMode);
            telemetry.addLine("Press b for red, x for blue, y adds delay, a removes delay");
            telemetry.addData("Start Delay", startDelay);
            telemetry.addData("collectorPower", collectorPower);
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
            } else if (gamepad1.leftStickButtonWasPressed()) {
                testingMode = true;
            }

            //Testing remove later
            if (gamepad2.yWasPressed()) {
                collectorPower += 0.05;
            } else if (gamepad2.aWasPressed()) {
                collectorPower -= 0.05;
            }
        } while (opModeInInit());

        runtime.reset();
        //imu.resetYaw();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            resetRuntime();
            while (runtime.seconds() < 0.25) {
                limelight.setTeam(teamID);
                limelight.process(telemetry);
                velLeft = (limelight.getRange() + 202.17 - 10) / 8.92124;
                velRight = (limelight.getRange() + 202.17 - 10) / 8.92124;
                telemetry.addData("Range", limelight.getRange());
                telemetry.update();
            }


            collectorBack.setPower(0.6);
            collectorFront.setPower(0.6);

            launchFlapLeft.setPosition(0.3);
            launchFlapRight.setPosition(0.4);
            sleep(startDelay*1000);


            pinpoint.odo.resetPosAndIMU();
            sleep(500);
            moveForward(2, 0.3);
            sleep(500);
            pinpoint.odo.resetPosAndIMU();
            turnTo(-20/turnOffset, 0.3);
            fireVolleySorted();
            sleep(500);
            pinpoint.odo.resetPosAndIMU();
            //turnTo(20/turnOffset, 0.3);
            //sleep(500);
            //pinpoint.odo.resetPosAndIMU();
            moveForward( 26,0.3);
            sleep(500);
            pinpoint.odo.resetPosAndIMU();
            turnTo(-80/turnOffset, 0.3);
            sleep(500);
            pinpoint.odo.resetPosAndIMU();
            moveForward( 12,0.3);
            sleep(500);
            pinpoint.odo.resetPosAndIMU();
            moveForward(5, 0.3);
            sleep(500);
            pinpoint.odo.resetPosAndIMU();
            moveForward(5,0.3);
            sleep(500);
            pinpoint.odo.resetPosAndIMU();
            moveForward(-22, -0.3);
            sleep(500);
            pinpoint.odo.resetPosAndIMU();
            turnTo(80/turnOffset,0.3);
            sleep(500);
            pinpoint.odo.resetPosAndIMU();
            moveForward(-26,-0.3);
            fireVolleySorted();
            sleep(500);
            pinpoint.odo.resetPosAndIMU();
            moveForward(5,0.3);
//            //rotateTo(-(aprilTags.getBearing()));
//            // if 20 look left
//            ElapsedTime turnLength = new ElapsedTime();
//            if (teamID == 20) {
////                while (turnLength.seconds() < 2) {
////                    limelight.setTeam(teamID);
////                    limelight.process(telemetry);
////                    turnToAprilTagLimelight();
////                }
//                ch.turn(-0.3,400);
//            } else {
////                while (turnLength.seconds() < 2) {
////                    limelight.setTeam(teamID);
////                    limelight.process(telemetry);
////                    turnToAprilTagLimelight();
////                }
//                ch.turn(0.3,400);
//            } // 450
//            runtime.reset();
//            while (runtime.seconds() < 0.25) {
//                limelight.setTeam(teamID);
//                limelight.process(telemetry);
//                velLeft = (limelight.getRange() + 202.17 - 10) / 8.92124;
//                velRight = (limelight.getRange() + 202.17 - 10) / 8.92124;
//                telemetry.addData("Range", limelight.getRange());
//                telemetry.update();
//            }
//
            // P is left

//            flipper.setPosition(0.525);
////            if (teamID == 20) {
////                turn(0.3, 200);
////            } else {
////                turn(0.3,200);
////            }
//            // moveForward(0.5, 400);
//            //moveForward(0.5,900);
//            //ms
//            //1200
//            shooterLeft.targetVelocity = 0;
//            shooterRight.targetVelocity = 0;
//            collectorFront.setPower(collectorPower);
//            collectorBack.setPower(collectorPower);
//
//            if (teamID == 24) {
//                ch.turn(-0.3,400);
//                ch.moveForward(0.5,900);
//                ch.turn(0.5,900);
//                ch.moveForward(0.5, 500);
//
//                sleep(2000);
//                flipper.setPosition(1);
//                sleep(250);
//                flipper.setPosition(0.525);
//                ch.moveForward(0.5, 100);
//                sleep(2000);
//                flipper.setPosition(0);
//                sleep(250);
//                flipper.setPosition(0.525);
//                ch.moveForward(0.5,250);
//
//                ch.moveForward(-0.5, 800);
//                ch.turn(-0.5,900);
////                moveForward(-0.5, 900);
////                turn(0.3,400);
//            } else {
//                ch.turn(0.3,400);
//                ch.moveForward(0.5,900);
//                ch.turn(-0.5,900);
//                ch.moveForward(0.5, 500);
//
//                sleep(2000);
//                flipper.setPosition(1);
//                sleep(250);
//                flipper.setPosition(0.525);
//                ch.moveForward(0.5, 100);
//                sleep(2000);
//                flipper.setPosition(0);
//                sleep(250);
//                flipper.setPosition(0.525);
//                ch.moveForward(0.5,120);
//
//                ch.moveForward(-0.5, 720);
//                ch.turn(-0.5,900);
//            }

//            if (teamID == 24)
//            {
//
//                turn(0.5,800);
//                //1300
//                moveForward(0.5, 300);
//                //ms
//                //900
//            }
//            else
//            {
//                turn(-0.5,800);
//                //pw
//                //-0.5
//                //ms
//                //1300
//                moveForward(0.5, 300);
//                //ms
//                //900
//                //200 low
//            }

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
    private void moveForward(double inches, double power) {
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
}
