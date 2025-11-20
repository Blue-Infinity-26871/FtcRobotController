package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Limelight Test", group="Concept")
public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevHubOrientationOnRobot = new RevHubOrientationOnRobot(com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles Orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(Orientation.getYaw());
        LLResult LLResult = limelight.getLatestResult();
        if (LLResult != null && LLResult.isValid()){
            Pose3D botPose = LLResult.getBotpose_MT2();
            telemetry.addData( "Tx", LLResult.getTx());
            telemetry.addData( "Ty", LLResult.getTy());
            telemetry.addData( "Ta", LLResult.getTa());
        }
    }
}
