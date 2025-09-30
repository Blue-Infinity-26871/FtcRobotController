package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odometry {
    private DcMotor leftEncoder, rightEncoder, strafeEncoder;

    private double x = 0, y = 0, heading = 0;

    private int lastLeft = 0, lastRight = 0, lastStrafe = 0;

    // Constants (adjust for your robot!)
    private static final double TICKS_PER_INCH = 8192.0 / (Math.PI * 4); // Example for REV encoder on 4" wheel
    private static final double TRACK_WIDTH = 14.5; // Inches between left/right encoders
    private static final double STRAFE_OFFSET = 7.5; // Optional

    public Odometry(HardwareMap hardwareMap) {
        leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
        strafeEncoder = hardwareMap.get(DcMotor.class, "strafeEncoder");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int strafePos = strafeEncoder.getCurrentPosition();

        int dL = leftPos - lastLeft;
        int dR = rightPos - lastRight;
        int dS = strafePos - lastStrafe;

        lastLeft = leftPos;
        lastRight = rightPos;
        lastStrafe = strafePos;

        double dLInches = dL / TICKS_PER_INCH;
        double dRInches = dR / TICKS_PER_INCH;
        double dSInches = dS / TICKS_PER_INCH;

        double dTheta = (dRInches - dLInches) / TRACK_WIDTH;
        heading += dTheta;

        double dx = dSInches - STRAFE_OFFSET * dTheta;
        double dy = (dLInches + dRInches) / 2.0;

        // Rotate (dx, dy) by current heading
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);

        double fieldX = dx * cos - dy * sin;
        double fieldY = dx * sin + dy * cos;

        x += fieldX;
        y += fieldY;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    @TeleOp(name = "Odometry Test", group = "Testing")
public class OdometryTest extends LinearOpMode {
    private Odometry odometry;

    @Override
    public void runOpMode() {
        odometry = new Odometry(hardwareMap); // <-- Use the Odometry class here

        waitForStart();

        while (opModeIsActive()) {
            odometry.update(); // <-- Keep updating position

            telemetry.addData("X (in)", odometry.getX());
            telemetry.addData("Y (in)", odometry.getY());
            telemetry.addData("Heading (rad)", odometry.getHeading());
            telemetry.update();
        }
    }
}
}

