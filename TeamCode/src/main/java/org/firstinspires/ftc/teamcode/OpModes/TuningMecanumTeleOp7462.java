/* Copyright (c) 2025 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.teamcode.GoalTag;
import org.firstinspires.ftc.teamcode.Shooter;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@Disabled
@TeleOp(name = "TUNING Mecanum TeleOp 7462", group = "Robot")
//@Disabled //comment this out when ready to add to android phone
public class TuningMecanumTeleOp7462 extends OpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    Shooter collectorBack;
    Shooter collectorFront;
    Shooter shooterLeft;
    Shooter shooterRight;

    Servo launchFlapLeft;
    Servo launchFlapRight;
    GoalTag goalTag;

    // Timers
    ElapsedTime timerLeft = new ElapsedTime();
    ElapsedTime timerRight = new ElapsedTime();

    Datalog tuningLog;

    // Just for tuning
    private double Kvelo;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;


    @Override
    public void init() {
        //hardwareMap is just so our code names can actually connect to what the android phone understands
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");
        launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //encoder uses ticks which is revolutions which tells how much to move

        //connecting our name for the orientation code to the android phone to tell it about the robot orientation (imu)
        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //we know the orientation of the robot by knowing the orientation of how the control hub
        //is placed on the robot with what orientation the logo is facing and what side
        //the usb ports are facing
        //because the logo or the usb could be facing up/down or right/left depending on how
        //they could fit the control hub on the robot

        goalTag = new GoalTag();
        goalTag.init(hardwareMap);

        collectorFront = new Shooter(hardwareMap,"collectorFront", true);
        collectorFront.setControllerValues(0.3,0.0243);
        collectorFront.targetVelocity = 0;

        collectorBack = new Shooter(hardwareMap,"collectorBack", true);
        collectorBack.setControllerValues(0.3,0.0243);
        collectorBack.targetVelocity = 0;

        shooterLeft = new Shooter(hardwareMap,"shooterLeft", false);
        shooterLeft.setControllerValues(0.3,0.0243);
        shooterLeft.targetVelocity = 20;

        shooterRight = new Shooter(hardwareMap,"shooterRight", true);
        shooterRight.setControllerValues(0.3,0.0243);

        timerLeft.reset();
        timerRight.reset();

        tuningLog = new Datalog("TuningLog");

    }
    public void moveAllMotors(double frontleftpower, double frontrightpower, double backleftpower, double backrightpower) {
        frontLeftDrive.setPower(frontleftpower);
        frontRightDrive.setPower(frontrightpower);
        backLeftDrive.setPower(backleftpower);
        backRightDrive.setPower(backrightpower);
    }

    //we are using the methods from OpMode and @Override is so that we can write our own stuff for this method
// Move to auto
    @Override
    public void init_loop() {
        goalTag.initProcess();
        telemetry.addData("Pattern", goalTag.getObelisk());
        telemetry.addData("team ID", goalTag.getGoalTagID());
        telemetry.update();
    }

    @Override
    public void loop() {
        goalTag.process();

        collectorFront.overridePower();
        collectorBack.overridePower();

        // Put line calculation for shooter velocity here
        // shooterRight.targetVelocity = goalTag.getRange();
        shooterRight.overridePower();
        // shooterLeft.targetVelocity = goalTag.getRange();
        shooterLeft.overridePower();

        telemetry.addData("shooterLeftCurrentVelocity", shooterLeft.getVelocity());
        telemetry.addData("shooterLeftTargetVelocity", shooterLeft.targetVelocity);
        telemetry.addData("shooterRightCurrentVelocity", shooterRight.getVelocity());
        telemetry.addData("shooterRightTargetVelocity", shooterRight.targetVelocity);
        telemetry.addData("GoalTagRange", goalTag.getRange());
        telemetry.addLine("Bumpers to shoot, a to turntotag");

        telemetry.update();

        // Testing Wiring
//        if(gamepad1.rightBumperWasPressed()) {
//            shooterRight.targetVelocity = 1;
//            shooterRight.overridePower();
//        }
//        else if(gamepad1.leftBumperWasPressed()) {
//            shooterLeft.overridePower();
//        }
//        else if(gamepad1.left_bumper) {
//            collectorBack.targetVelocity = 1;
//            collectorBack.overridePower();
//        }
//        else if(gamepad1.right_bumper) {
//            collectorFront.targetVelocity = 1;
//            collectorFront.overridePower();
//        }

        // Tuning
//        if (gamepad1.xWasPressed()) {
//            Kvelo += 0.005;
//            shooterLeft.setControllerValues(0,Kvelo);
//        } else if (gamepad1.yWasPressed()) {
//            Kvelo -= 0.005;
//            shooterLeft.setControllerValues(0,Kvelo);
//        }
        // Driver Controls
        if (gamepad1.leftBumperWasPressed() && shooterLeft.atSpeed()) {
            launchFlapLeft.setPosition(0);
            timerLeft.reset();
        }
        if (gamepad1.rightBumperWasPressed() && shooterRight.atSpeed()) {
            launchFlapRight.setPosition(0.7);
            timerRight.reset();
        }
        if (gamepad1.left_trigger == 1) {
            tuningLog.goalBool.set(true);
            tuningLog.goalRange.set(goalTag.getRange());
            tuningLog.targetVelocity.set(shooterLeft.targetVelocity);
            tuningLog.writeLine();
       }
//        if (gamepad1.right_trigger == 1) {
//            tuningLog.goalBool.set(true);
//            tuningLog.goalRange.set(goalTag.getRange());
//            tuningLog.targetVelocity.set(shooterRight.targetVelocity);
//            tuningLog.writeLine();
//        }
        if (gamepad1.aWasPressed()) {
            shooterLeft.targetVelocity -= 0.25;
//            shooterRight.targetVelocity -= 0.25;
        }
        if (gamepad1.yWasPressed()) {
            shooterLeft.targetVelocity += 0.25;
//            shooterRight.targetVelocity += 0.25;
        }

        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.xWasPressed()) {
            turnToAprilTag();
        }
        // Launch Flap Reset
        if (timerLeft.seconds() > 2) {
            launchFlapLeft.setPosition(0.3);
//            shooterLeft.targetVelocity = 0;
        }
        if (timerRight.seconds() > 2) {
            launchFlapRight.setPosition(0.4);
//            shooterRight.targetVelocity = 0;
        }
    }
    public void turnToAprilTag() {
        if (goalTag.getBearing() > 0.6 || goalTag.getBearing() < -0.6) {
            if (goalTag.getBearing() > 0.6) { // rotate left
                moveAllMotors(-0.2,0.2,-0.2,0.2);
            } else if (goalTag.getBearing() < -0.6) { // rotate right
                moveAllMotors(0.2,-0.2,0.2,-0.2);

            }
        }
    }
    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
    public static class Datalog {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField goalBool = new Datalogger.GenericField("goalBool");
        public Datalogger.GenericField goalRange = new Datalogger.GenericField("goalRange");
        public Datalogger.GenericField targetVelocity = new Datalogger.GenericField("targetVelocity");

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
                            goalBool,
                            targetVelocity,
                            goalRange
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
