package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; 

@TeleOp(name="Test", group="Concept") 
public class Test extends LinearOpMode {

private DcMotor M1 = null; 

@Override
public void runOpMode() {

M1 = hardwareMap.get(DcMotor.class, "M1"); 



waitForStart();


while (opModeIsActive()) {

M1.setPower(gamepad1.left_stick_y); 
}
}
}