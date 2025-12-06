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
    private double velLeft = 0;
    private double velRight = 0;

    private double currentX;
    private double currentY;
    private double currentAngle;
    private int turnSign;
    ElapsedTime runtime = new ElapsedTime();
    GoalTagLimelight limelight;
    private int startDelay = 0;
    private int teamID;
    private boolean testingMode = false;

    private boolean shooting = false;
    private double collectorPower = 0.53;
    private static double turnOffset = 3.8;

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

        pinpoint.odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,0));

        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap,telemetry);

        pinpoint.odo.resetPosAndIMU();
        pinpoint.odo.recalibrateIMU();

        Pose2D pose = pinpoint.odo.getPosition();

        GlobalStorage.setPattern(null);
        GlobalStorage.setAlliance(-1);


        do {
            pinpoint.odo.update();
            limelight.readObelisk(telemetry);
            GlobalStorage.setPattern(limelight.getObelisk());

            telemetry.addData("Pattern", limelight.getObelisk());
            telemetry.addData("Is Tag Recent", limelight.seeObelisk);
            telemetry.addData("team ID", teamID);
            telemetry.addData("Testing Mode", testingMode);
            telemetry.addLine("Press b for red, x for blue, y adds delay, a removes delay");
            telemetry.addData("Start Delay", startDelay);
            telemetry.addData("collectorPower", collectorPower);

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

            telemetry.addData("Heading Scalar", pinpoint.odo.getYawScalar());
            telemetry.addData("Initial X", "%.2f", pinpoint.odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("Initial Y", "%.2f", pinpoint.odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("Initial Heading (deg) MAKE SURE ITS 0", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Status", pinpoint.odo.getDeviceStatus());
            telemetry.update();
        } while (opModeInInit());

        if (teamID == 24) {
            turnSign = -1;
        } else {
            turnSign = 1;
        }
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            limelight.setTeam(teamID);
            sleep(startDelay*1000);

            collectorBack.setPower(collectorPower);
            collectorFront.setPower(collectorPower);

            launchFlapLeft.setPosition(0.3);
            launchFlapRight.setPosition(0.4);

            moveX(2, 0.3);

            turnTo(turnSign*20, 0.25);

            if (!testingMode) {fireVolleySorted();}

            flipper.setPosition(0.525);
            moveX( 24,0.3);
            turnTo(turnSign*90, 0.25);

            moveY( turnSign*15,0.3);
            sleep(500);
            flipper.setPosition(0);
            sleep(250);
            flipper.setPosition(0.525);
            moveY(turnSign*20, 0.3);
            sleep(500);
            flipper.setPosition(1);
            sleep(250);
            flipper.setPosition(0.525);
            sleep(500);
            moveY(turnSign*25,0.3);
            sleep(500);
            moveY(turnSign*5, -0.3);

            turnTo(turnSign*20,-0.25);

            moveX(2,-0.3);
            if (!testingMode){fireVolleySorted();}
            moveX(20,0.3);

            break;
        }
    }
    public void fireVolleySorted() {
        runtime.reset();
        while (runtime.seconds() < 0.1) {
            limelight.process(telemetry);
            velLeft = (limelight.getRange() + 202.17 - 10) / 8.92124;
            velRight = (limelight.getRange() + 202.17 - 10) / 8.92124;
        }
        if (limelight.getObelisk().equals("PGP") && !testingMode) {
            fireShooterLeft(velLeft);
            fireShooterRight(velRight);
            flipper.setPosition(1);
            sleep(250);
            fireShooterLeft(velLeft);
        } else if (limelight.getObelisk().equals("GPP") && !testingMode) {
            fireShooterRight(velRight);
            fireShooterLeft(velLeft);
            flipper.setPosition(1);
            sleep(500);
            fireShooterLeft(velLeft);
        } else if (limelight.getObelisk().equals("PPG") && !testingMode) {
            fireShooterLeft(velLeft);
            sleep(250);
            flipper.setPosition(1);
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
