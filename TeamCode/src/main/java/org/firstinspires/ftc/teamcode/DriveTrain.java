package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp 2025/26", group="Linear OpMode")

public class DriveTrain extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor M1 = null;
    private DcMotor M2 = null;
    private DcMotor M3 = null;
    private DcMotor M4 = null;
    private DcMotor M5 = null;
    private DcMotor M6 = null;
    private DcMotor M7 = null;
    private DcMotor M8 = null;
    private Servo S1 = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        M1  = hardwareMap.get(DcMotor.class, "M1");
        M2  = hardwareMap.get(DcMotor.class, "M2");
        M3 = hardwareMap.get(DcMotor.class, "M3");
        M4 = hardwareMap.get(DcMotor.class, "M4");
        M5 = hardwareMap.get(DcMotor.class, "M5");
        M6 = hardwareMap.get(DcMotor.class, "M6");
        M7 = hardwareMap.get(DcMotor.class, "M7");
        M8 = hardwareMap.get(DcMotor.class, "M8");
        S1 = hardwareMap.get(Servo.class, "S1");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        M1.setDirection(DcMotor.Direction.REVERSE);
        M2.setDirection(DcMotor.Direction.REVERSE);
        M5.setDirection(DcMotor.Direction.REVERSE);
        M7.setDirection(DcMotor.Direction.REVERSE);

        // Run until the end of the match (driver presses STOP)
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            double max;
            double hatUp = gamepad1.dpad_up ? 1.0 : 0.0;
            double hatDown = gamepad1.dpad_down ? 1.0 : 0.0;
            double hatRight = gamepad1.dpad_right ? 1.0 : 0.0;
            double hatLeft = gamepad1.dpad_left ? 1.0 : 0.0;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            double leftFrontPower  = (axial + lateral + yaw);
            double rightFrontPower = (axial - lateral - yaw);
            double leftBackPower   = (axial - lateral + yaw);
            double rightBackPower  = (axial + lateral - yaw);

            // double armRotatorPower = 0.6*-gamepad2.left_stick_y;


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }



            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.



            /*

            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            M1.setPower(leftFrontPower);
            M2.setPower(leftBackPower);
            M3.setPower(rightFrontPower);
            M4.setPower(rightBackPower);
            M5.setPower(gamepad1.right_trigger);

            //Driver 2
            double aPower = gamepad2.a ? 1.0 : 0.0;
            double halfPower = gamepad2.left_bumper ?  0.5 : 0.0;
            double eightyPower = gamepad2.right_bumper ?  0.8 : 0.0;
            double bPower = gamepad2.b ? 1.0 : 0.0;
            double aPowerBackward = gamepad2.dpad_up ? 1.0 : 0.0;
            double xPower = gamepad2.x ? 1.0 : 0.0;
            double yPower = gamepad2.y ? -1.0 : 0.0;
            double m7Power = aPower+halfPower+eightyPower+0.3*yPower-0.4*xPower;
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
            //M6.setPower(-gamepad2.left_stick_y*0.1);

            //intake.setPower(intakePower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

        }
    }}
