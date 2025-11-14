package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class GoalTagLimelight {
    Limelight3A limelight;
    //Pipeline 0 is 20(blue) pipeline 1 is 24(red)

    public void init(HardwareMap hardwareMap, Telemetry telemetry, int id) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        telemetry.setMsTransmissionInterval(11);
        setTeam(id);
        limelight.start(); // This tells Limelight to start looking!
    }

    public void loop(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double range = Math.sqrt(Math.pow((ty-y),2) + Math.pow((tx-x),2));

                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            }

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
    public void setTeam(int id) {
        if (id == 24) {
            limelight.pipelineSwitch(0);
        } else if (id == 20) {
            limelight.pipelineSwitch(1);
        }
    }
}
