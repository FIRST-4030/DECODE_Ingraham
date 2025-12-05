package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Pinpoint;

@TeleOp(name="OdometryDiagnosticsLinear", group="Demo")
public class OdometryDiagnosticsLinear extends LinearOpMode {

    Chassis ch;
    Pinpoint pinpoint;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        ch = new Chassis(hardwareMap);

        pinpoint = new Pinpoint(hardwareMap, ch, telemetry,  false);

        pinpoint.setEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0,AngleUnit.DEGREES,0));

        resetRuntime();

        pinpoint.odo.resetPosAndIMU();

        pinpoint.odo.recalibrateIMU();  // this takes 0.25 seconds to initialize

        while (runtime.seconds() < 0.3) {
            telemetry.addData("Status", pinpoint.odo.getDeviceStatus());
            telemetry.update();
        }

        do {
            pinpoint.odo.update();

            telemetry.addData("Initial X", "%.2f", pinpoint.odo.getPosX(DistanceUnit.MM));
            telemetry.addData("Initial Y", "%.2f", pinpoint.odo.getPosY(DistanceUnit.MM));
            telemetry.addData("Initial Heading (deg)", "%.1f", pinpoint.odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Status", pinpoint.odo.getDeviceStatus());
            telemetry.update();

        } while (opModeInInit());

        while (opModeIsActive()) {

            pinpoint.odo.update();

            Pose2D pose = pinpoint.odo.getPosition();
            double heading = pose.getHeading(AngleUnit.DEGREES);

            ch.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("X", "%.2f", pinpoint.odo.getPosX(DistanceUnit.MM));
            telemetry.addData("Y", "%.2f", pinpoint.odo.getPosY(DistanceUnit.MM));
            telemetry.addData("Get Heading (deg)", "%.1f", pinpoint.odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pose Heading (deg)", "%.1f", heading);
            telemetry.addData("Status", pinpoint.odo.getDeviceStatus());
            telemetry.update();
        }
    }
}
