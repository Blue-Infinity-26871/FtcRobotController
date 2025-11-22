package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PidTuningExample extends LinearOpMode {
    private DcMotorEx m5;
    public DcMotorEx shooter;
    private static final int REV_HD_MOTOR_TICKS_PER_ROTATION = 28;
    private static final double SHOOTER_MAX_RPM = 5500;
    double shooterTargetVelocity = 0;
    // 5500 / 60 = rotations per second
    public double  rpmToTicksPerSecond(double rpm) {
        return rpm / 60 * REV_HD_MOTOR_TICKS_PER_ROTATION;
    }

    public double ticksPerSecondToRPM(double tps) {
        return tps / REV_HD_MOTOR_TICKS_PER_ROTATION * 60;
    }



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m5 = hardwareMap.get(DcMotorEx.class, "M7");

        // Assign motors to more friendly names
        shooter = m5;
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            PIDFCoefficients currentPIDFCoefficients = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            if (currentPIDFCoefficients.p != X25Config.SHOOTER_PID.p ||
                    currbrentPIDFCoefficients.i != X25Config.SHOOTER_PID.i ||
                    currentPIDFCoefficients.d != X25Config.SHOOTER_PID.d ||
                    currentPIDFCoefficients.f != X25Config.SHOOTER_PID.f){
                shooter.setVelocityPIDFCoefficients(X25Config.SHOOTER_PID.p, X25Config.SHOOTER_PID.i, X25Config.SHOOTER_PID.d, X25Config.SHOOTER_PID.f);
            }

            if (gamepad1.a) {
                shooterTargetVelocity = rpmToTicksPerSecond(SHOOTER_MAX_RPM);
            } else if (gamepad1.b) {
                shooterTargetVelocity = 0;
            } else if (gamepad1.y) {
                shooterTargetVelocity = Math.min(rpmToTicksPerSecond(SHOOTER_MAX_RPM), shooterTargetVelocity + rpmToTicksPerSecond(100));
            } else if (gamepad1.x) {
                shooterTargetVelocity = Math.max(0, shooterTargetVelocity - rpmToTicksPerSecond(100));
            }

            telemetry.addData("shooterTargetVelocityRPM", ticksPerSecondToRPM(shooterTargetVelocity));
            telemetry.addData("shooterVelocityRPM", ticksPerSecondToRPM(shooter.getVelocity()));
            telemetry.update();

            shooter.setVelocity(shooterTargetVelocity);
        }
    }
}