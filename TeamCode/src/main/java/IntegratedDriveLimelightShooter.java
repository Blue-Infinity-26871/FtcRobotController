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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Integrated TeleOp for drivetrain, Limelight, IMU, and shooter PID
 */
@TeleOp(name="Integrated Drive + Limelight + Shooter", group="Integration")
public class IntegratedDriveLimelightShooter extends LinearOpMode {

    // --- Drivetrain + Servo ---
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor M1, M2, M3, M4, M5, M6, M7, M8;
    private Servo S1;

    // --- Limelight + IMU ---
    private Limelight3A limelight;
    private IMU imu;

    // --- Shooter PID setup ---
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // --- Initialize hardware ---
        M1 = hardwareMap.get(DcMotor.class, "M1");
        M2 = hardwareMap.get(DcMotor.class, "M2");
        M3 = hardwareMap.get(DcMotor.class, "M3");
        M4 = hardwareMap.get(DcMotor.class, "M4");
        M5 = hardwareMap.get(DcMotor.class, "M5");
        M6 = hardwareMap.get(DcMotor.class, "M6");
        M7 = hardwareMap.get(DcMotor.class, "M7");
        M8 = hardwareMap.get(DcMotor.class, "M8");
        S1 = hardwareMap.get(Servo.class, "S1");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        shooter = hardwareMap.get(DcMotorEx.class, "M7"); // Shooter motor

        // --- IMU setup ---
        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(hubOrientation));

        // --- Motor directions ---
        M1.setDirection(DcMotor.Direction.REVERSE);
        M2.setDirection(DcMotor.Direction.REVERSE);
        M5.setDirection(DcMotor.Direction.REVERSE);
        M7.setDirection(DcMotor.Direction.REVERSE);

        // --- Shooter setup ---
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Start Limelight ---
        limelight.start();

        telemetry.addLine("Initialized — Ready to start");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // ========== DRIVE SECTION ==========
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            M1.setPower(leftFrontPower);
            M2.setPower(leftBackPower);
            M3.setPower(rightFrontPower);
            M4.setPower(rightBackPower);

            // ========== LIMELIGHT SECTION ==========
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
            }

            // ========== SHOOTER PID SECTION ==========
            PIDFCoefficients pidf = new PIDFCoefficients(80, 10, 0, 12);
            shooter.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);


            shooter.setVelocity(shooterTargetVelocity);

            double aPower = gamepad2.a ? 1.0 : 0.0;
            double LTurretPower = gamepad2.left_bumper ?  0.8 : 0.0;
            double eightyPower = gamepad2.right_bumper ?  0.8 : 0.0;
            double bPower = gamepad2.b ? 1.0 : 0.0;
            double aPowerBackward = gamepad2.dpad_up ? 1.0 : 0.0;
            double xPower = gamepad2.x ? 1.0 : 0.0;
            double yPower = gamepad2.y ? -1.0 : 0.0;
            double m7Power = aPower+LTurretPower+eightyPower+0.3*yPower-0.4*xPower;
            M5.setPower(bPower);
            M6.setPower(gamepad2.left_trigger);
            M6.setPower(-gamepad2.right_trigger);
            M7.setPower(m7Power);
            M8.setPower(xPower);
            M8.setPower(yPower);
            if (m7Power > 0.4) {
                S1.setPosition(0.5);
            } else {
                S1.setPosition(0);
            }


            // ========== TELEMETRY ==========
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Shooter Target (RPM)", ticksPerSecondToRPM(shooterTargetVelocity));
            telemetry.addData("Shooter Actual (RPM)", ticksPerSecondToRPM(shooter.getVelocity()));
            telemetry.update();
        }
    }
}
