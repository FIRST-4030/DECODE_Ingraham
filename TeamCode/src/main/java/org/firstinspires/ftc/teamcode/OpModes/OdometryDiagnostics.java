package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Pinpoint;
@Disabled

@TeleOp(name="OdometryDiagnostics", group="Demo")
public class OdometryDiagnostics extends OpMode {

    Chassis ch;
    Pinpoint pinpoint;
    double initialHeading;

    @Override
    public void init() {

        ch = new Chassis(hardwareMap);

        pinpoint = new Pinpoint(hardwareMap, ch, telemetry,  false);

        pinpoint.setEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0,AngleUnit.DEGREES,0));
        pinpoint.odo.recalibrateIMU();
        pinpoint.odo.resetPosAndIMU();
    }

    @Override
    public void init_loop() {

        pinpoint.odo.update();

        telemetry.addData("X offset", pinpoint.odo.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset", pinpoint.odo.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Heading Scalar", pinpoint.odo.getYawScalar());
        telemetry.addData("Initial X", "%.2f", pinpoint.odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Initial Y", "%.2f", pinpoint.odo.getPosY(DistanceUnit.MM));
        telemetry.addData("Initial Heading (deg)", "%.1f", pinpoint.odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Status", pinpoint.odo.getDeviceStatus());
        telemetry.update();
    }

    @Override
    public void start() {
        initialHeading = pinpoint.odo.getHeading(AngleUnit.RADIANS);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {

        Pose2D pose = pinpoint.odo.getPosition();
        pinpoint.odo.update();

        ch.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        double encoderX = pinpoint.odo.getEncoderX();
        double encoderY = pinpoint.odo.getEncoderY();
        double currentX = pinpoint.odo.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.odo.getPosY(DistanceUnit.INCH);
        double currentHeading = pose.getHeading(AngleUnit.DEGREES);
        double headingError = initialHeading - currentHeading;

        // Normalize heading error
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        telemetry.addLine(String.format("Heading error: %6.2f", Math.toDegrees(headingError)));
        telemetry.addLine(String.format("Encoder X: %6.2f", encoderX));
        telemetry.addLine(String.format("Encoder Y: %6.2f", encoderY));
        telemetry.addLine(String.format("Current X: %6.2f", currentX));
        telemetry.addLine(String.format("Current Y: %6.2f", currentY));
        telemetry.addLine(String.format("Current Heading: %.1f", currentHeading));
        telemetry.addLine(String.format("Current Velocity: %6.2f", pinpoint.odo.getPosX(DistanceUnit.INCH)));
        telemetry.update();
    }
}