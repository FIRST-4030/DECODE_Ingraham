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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
import org.firstinspires.ftc.teamcode.SensorGoBildaPinpoint;
import org.firstinspires.ftc.teamcode.Shooter;

@Autonomous(name="Mecanum Auto Far Move Off Line", group="Linear OpMode")
public class MecanumAutoFarMoveOffLine extends LinearOpMode {

    // Declare OpMode members.
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    Shooter shooterLeft;
    Shooter shooterRight;
    Servo launchFlapLeft;
    Servo launchFlapRight;
    Shooter collectorBack;
    Shooter collectorFront;
    Servo flipper;
    private int velLeft = 35;
    private int velRight = 34;

    ElapsedTime runtime = new ElapsedTime();
    public static int decimation = 3;
    public static double power = 0.7;
    double yawImu;
    YawPitchRollAngles orientation;

    //GoalTag goalTag;
    GoalTagLimelight limelight;
    private int startDelay = 0;
    private int teamID;

    private boolean shooting = false;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    SensorGoBildaPinpoint pinpoint;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        collectorFront = new Shooter(hardwareMap,"collectorFront", false);

        collectorBack = new Shooter(hardwareMap,"collectorBack", false);

        flipper = hardwareMap.get(Servo.class, "flipper");

        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);
        shooterRight = new Shooter(hardwareMap, "shooterRight", false);

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");

        launchFlapRight= hardwareMap.get(Servo.class, "launchFlapRight");

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

//        goalTag = new GoalTag();
//        goalTag.init(hardwareMap);
        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap,telemetry);


        GlobalStorage.setPattern(null);
        GlobalStorage.setAlliance(-1);

//        pinpoint = new SensorGoBildaPinpoint();
//        pinpoint.initOdometry(hardwareMap, 0, 0, 0);

        do {
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
        imu.resetYaw();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            sleep(startDelay*1000);


            flipper.setPosition(0.525);

            moveForward(0.5, 400);
            //moveForward(0.5,900);
            //ms
            //1200
            shooterLeft.targetVelocity = 0;
            shooterRight.targetVelocity = 0;
            collectorFront.setPower(0.6);
            collectorBack.setPower(0.6);


//

            break;
        }
    }

    private void moveForward(double power, double mseconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.milliseconds() < mseconds) {
            frontLeftDrive.setPower(power);
            backLeftDrive.setPower(power);
            frontRightDrive.setPower(power);
            backRightDrive.setPower(power);
        }

        stopMotors();
    }
    private void turn(double power, double mseconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.milliseconds() < mseconds) {
            frontLeftDrive.setPower(power);
            backLeftDrive.setPower(power);
            frontRightDrive.setPower(-power);
            backRightDrive.setPower(-power);
        }

        stopMotors();
    }

    private void rotateTo(double targetAngle) {
        double Kp = 0.03;  // Proportional gain (tune this)
        double Kd = 0.0;  // derivative gain
        double minPower = 0.3;
        double maxPower = 0.5;
        double tolerance = 3.0; // degrees
        double lastError = 0;
        double derivative;
        double currentAngle, error, turnPower;

        long lastTime = System.nanoTime();

        while (true) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            error = -targetAngle - currentAngle;
            error = (error + 540) % 360 - 180; // Wrap error to [-180, 180] range

            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;
            lastTime = now;

            derivative = (error - lastError) / deltaTime;
            lastError = error;

            if (Math.abs(error) < tolerance) break;

            turnPower = Kp * error + Kd * derivative;

            // Enforce minimum power
            if (Math.abs(turnPower) < minPower) {
                turnPower = Math.signum(turnPower) * minPower;
            }
            // Clamp maximum power
            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

            telemetry.addData("Target (deg)", "%.2f", targetAngle);
            telemetry.addData("Current (deg)", "%.2f", currentAngle);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Turn Power", "%.2f", turnPower);
            telemetry.update();

            frontLeftDrive.setPower(-turnPower);
            backLeftDrive.setPower(-turnPower);
            frontRightDrive.setPower(turnPower);
            backRightDrive.setPower(turnPower);
        }
    }

    private void stopMotors() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }


}
