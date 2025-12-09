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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
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
@TeleOp(name = "Mecanum TeleOp 7462", group = "Robot")
//@Disabled //comment this out when ready to add to android phone
public class MecanumTeleOp7462 extends OpMode {
    GoalTagLimelight limelight;
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
    Servo flipper;
    //GoalTag goalTag;

    // Timers
    ElapsedTime timerLeft = new ElapsedTime();
    ElapsedTime timerRight = new ElapsedTime();
    ElapsedTime timerFlipper = new ElapsedTime();

    // Just for tuning
    private double Kvelo;
    private double idlePower = 20;
    private double lastError = 0;
    private double frontVel = 15;
    private double backVel = 15;
    private double kP = 0.14;
    private double kD = 0.038;
    private boolean leftIsRunning;
    private boolean rightIsRunning;
    private boolean emergencyMode = false;
    private double maxPower = 1.0;
    private double maxSpeed = 1.0;
    private double collectorPower = 0.5;

    // This declares the IMU needed to get the current direction the robot is facing
//    IMU imu;


    @Override
    public void init() {
        //hardwareMap is just so our code names can actually connect to what the android phone understands
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");
        launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");
        flipper = hardwareMap.get(Servo.class, "flipper");

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
//        imu = hardwareMap.get(IMU.class, "imu");
//        // This needs to be changed to match the orientation on your robot
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
//                RevHubOrientationOnRobot.LogoFacingDirection.UP;
//        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
//
//        RevHubOrientationOnRobot orientationOnRobot = new
//                RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //we know the orientation of the robot by knowing the orientation of how the control hub
        //is placed on the robot with what orientation the logo is facing and what side
        //the usb ports are facing
        //because the logo or the usb could be facing up/down or right/left depending on how
        //they could fit the control hub on the robot

//        goalTag = new GoalTag();
//        goalTag.init(hardwareMap);

        collectorFront = new Shooter(hardwareMap,"collectorFront", false);
        //collectorFront.setControllerValues(0.3,0.0243);
        //collectorFront.targetVelocity = frontVel;

        collectorBack = new Shooter(hardwareMap,"collectorBack", false);
//        collectorBack.setControllerValues(0.3,0.0243);
//        collectorBack.targetVelocity = backVel;

        shooterLeft = new Shooter(hardwareMap,"shooterLeft", true);
        shooterLeft.setControllerValues(0.3,0.0243);

        shooterRight = new Shooter(hardwareMap,"shooterRight", false);
        shooterRight.setControllerValues(0.3,0.0243);

        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap, telemetry);

        if ((GlobalStorage.getAlliance() != -1)) {
            //goalTag.targetAprilTagID = GlobalStorage.getAlliance();
            limelight.setTeam(GlobalStorage.getAlliance());
        }
        timerLeft.reset();
        timerRight.reset();
        timerFlipper.reset();




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
        telemetry.addData("Pattern", limelight.getObelisk());
        telemetry.addData("team ID", limelight.getID());

        telemetry.addLine("Press b for red, x for blue");
        telemetry.update();
        if (gamepad1.bWasPressed()) {
//            goalTag.targetAprilTagID = 24;
            limelight.setTeam(24);
        } else if (gamepad1.xWasPressed()) {
            //goalTag.targetAprilTagID = 20;
            limelight.setTeam(20);
        }
    }

    @Override
    public void start() {
        collectorFront.setPower(0.6);
        collectorBack.setPower(0.6);
    }

    @Override
    public void loop() {
        //goalTag.process();
        limelight.process(telemetry);

        shooterRight.overridePower();
        shooterLeft.overridePower();
        //Testing remove this
//        collectorFront.setPower(collectorPower);
//        collectorBack.setPower(collectorPower);


        telemetry.addData("shooterLeftCurrentVelocity", shooterLeft.getVelocity());
        telemetry.addData("shooterLeftTargetVelocity", shooterLeft.targetVelocity);
        telemetry.addData("shooterRightCurrentVelocity", shooterRight.getVelocity());
        telemetry.addData("shooterRightTargetVelocity", shooterRight.targetVelocity);
        telemetry.addData("collectorFrontCurrentPower", collectorFront.getPower());
        telemetry.addData("collectorBackCurrentPower", collectorBack.getPower());
        telemetry.addData("Kp", kP);
        telemetry.addData("KD", kD);
        telemetry.addData("TimerLeft", timerLeft.seconds());
        telemetry.addLine("Bumpers to shoot, a to turntotag");



        // Driver Controlstelemetry.addData("Is Tag Recent", limelight.seeObelisk);
        if (gamepad1.leftBumperWasPressed() && (limelight.isDataCurrent || emergencyMode)) {
            // do math here
            shooterLeft.targetVelocity = (limelight.getRange() + 202.17 - 10) / 8.92124;
            //shooterLeft.targetVelocity = 0.1067*limelight.getRange()+24.336;
            leftIsRunning = true;
            timerLeft.reset();
        }
        if (gamepad1.rightBumperWasPressed() && (limelight.isDataCurrent || emergencyMode)) {
            // do math here
            shooterRight.targetVelocity = (limelight.getRange() + 202.17 - 10) / 8.92124;
            //shooterRight.targetVelocity = 0.1067*limelight.getRange()+24.336;
            rightIsRunning = true;
            timerRight.reset();
        }
        if (gamepad2.dpadLeftWasPressed()) {
            flipper.setPosition(1);
            timerFlipper.reset();
        }
        if (gamepad2.dpadRightWasPressed()) {
            flipper.setPosition(0.1);
            timerFlipper.reset();
        }
        if (gamepad2.dpadUpWasPressed()) {
            collectorBack.setPower(-0.6);
            collectorFront.setPower(-0.6);
        }
        if (gamepad2.dpadUpWasReleased()) {
            collectorFront.setPower(0.6);
            collectorBack.setPower(0.6);
        }
        if (gamepad1.a && limelight.isDataCurrent) {
            turnToAprilTagLimelight();
        }
//        if (gamepad2.yWasPressed()) {
//            collectorPower += 0.05;
//
//        } else if (gamepad2.aWasPressed()) {
//            collectorPower -= 0.05;
//        } //else if (gamepad2.bWasPressed()) {
//            kD += 0.0005;
//        } else if (gamepad2.xWasPressed()) {
//            kD -= 0.0005;
//        }
        // Parking mode
        if (gamepad1.right_trigger == 1) {
            maxPower = 0.5;
            maxSpeed = 0.5;
        } else {
            maxPower = 1.0;
            maxSpeed = 1.0;
        }
        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Shoot when at speed
        if (leftIsRunning) {
            if (shooterLeft.atSpeed()) {
                timerLeft.reset();
                launchFlapLeft.setPosition(0);
                leftIsRunning = false;
            }
        }
        if (rightIsRunning) {
            if (shooterRight.atSpeed()) {
                timerRight.reset();
                launchFlapRight.setPosition(0.7);
                rightIsRunning = false;
            }
        }
        // Servo Reset
        if (timerLeft.seconds() > 0.5 && !leftIsRunning) {
            launchFlapLeft.setPosition(0.3);
            shooterLeft.targetVelocity = idlePower;
        }
        if (timerRight.seconds() > 0.5 && !rightIsRunning) {
            launchFlapRight.setPosition(0.4);
            shooterRight.targetVelocity = idlePower;
        }
        if (timerFlipper.seconds() > 0.25) {
            flipper.setPosition(0.525);
        }
        // If camera not working press this and shoot near point of close V.
        if (gamepad1.leftStickButtonWasPressed()) {
            shooterLeft.targetVelocity = 30;
            shooterRight.targetVelocity = 30;
            idlePower = 30;
            emergencyMode = true;
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
    private void turnTo(double variance, double setPoint) {
        double currentAngle = limelight.getTx();
        double error = setPoint - currentAngle;

        if (Math.abs(error) > variance) {
            double power = kP*error;

            telemetry.addData("turn power", power);
            moveAllMotors(-power,power,-power,power);
//            if (error > rightBound) { // rotate left
//                moveAllMotors(-power,power,-power,power);
//            } else if (error < leftBound) { // rotate right
//                moveAllMotors(power,-power,power,-power);
//            }
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

        // make this slower for outreaches

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
}
