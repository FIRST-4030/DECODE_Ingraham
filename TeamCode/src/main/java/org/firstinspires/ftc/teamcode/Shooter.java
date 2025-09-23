package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shooter {
    private DcMotorEx shooter;
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setVelocityGivenRange(double range) {
        double velocity = range; // Do math here
        shooter.setVelocity(velocity);
    }
    public double getPower() {
        return shooter.getPower();
    }
    public double getVelocity() {
        return shooter.getVelocity();
    }
}
