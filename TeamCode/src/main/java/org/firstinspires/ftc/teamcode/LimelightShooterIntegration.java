package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Limelight + Shooter PID Integration", group="Integration")
public class LimelightShooterIntegration extends LinearOpMode {

    // --- Limelight + IMU ---
    private Limelight3A limelight;
    private IMU imu;

    // --- Shooter Motor + PID ---
    private DcMotorEx shooter;
    private static final int REV_HD_MOTOR_TICKS_PER_ROTATION = 28;
    private static final double SHOOTER_MAX_RPM = 5500;
    private double shooterTargetVelocity = 0;

    // --- Helper Methods ---
    public double rpmToTicksPerSecond(double rpm) {
        return rpm / 60.0 * REV_HD_MOTOR_TICKS_PER_ROTATION;
    }

    public double ticksPerSecondToRPM(double tps) {
        return tps / REV_HD_MOTOR_TICKS_PER_ROTATION * 60.0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // --- Initialize hardware ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        shooter = hardwareMap.get(DcMotorEx.class, "M7");

        // IMU orientation setup
        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(hubOrientation));

        // Shooter motor config
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start Limelight
        limelight.start();

        telemetry.addLine("Initialized — ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- LIMELIGHT SECTION ---
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());

            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
            }

            // --- SHOOTER PID SECTION ---
            PIDFCoefficients currentPIDF = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            if (currentPIDF.p != X25Config.SHOOTER_PID.p ||
                    currentPIDF.i != X25Config.SHOOTER_PID.i ||
                    currentPIDF.d != X25Config.SHOOTER_PID.d ||
                    currentPIDF.f != X25Config.SHOOTER_PID.f) {
                shooter.setVelocityPIDFCoefficients(
                        X25Config.SHOOTER_PID.p,
                        X25Config.SHOOTER_PID.i,
                        X25Config.SHOOTER_PID.d,
                        X25Config.SHOOTER_PID.f
                );
            }

            // Gamepad shooter control
            if (gamepad1.a) {
                shooterTargetVelocity = rpmToTicksPerSecond(SHOOTER_MAX_RPM);
            } else if (gamepad1.b) {
                shooterTargetVelocity = 0;
            } else if (gamepad1.y) {
                shooterTargetVelocity = Math.min(
                        rpmToTicksPerSecond(SHOOTER_MAX_RPM),
                        shooterTargetVelocity + rpmToTicksPerSecond(100)
                );
            } else if (gamepad1.x) {
                shooterTargetVelocity = Math.max(
                        0,
                        shooterTargetVelocity - rpmToTicksPerSecond(100)
                );
            }

            shooter.setVelocity(shooterTargetVelocity);

            // Combined telemetry
            telemetry.addData("Shooter Target (RPM)", ticksPerSecondToRPM(shooterTargetVelocity));
            telemetry.addData("Shooter Actual (RPM)", ticksPerSecondToRPM(shooter.getVelocity()));
            telemetry.update();
        }
    }
}
