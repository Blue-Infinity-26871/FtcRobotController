package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; 

@TeleOp(name="Basic_Drive_Train_Expansion_Hub", group="Concept") 
public class BasicDriveTrainExpansionHub extends LinearOpMode {

private DcMotor M1 = null; 
private DcMotor M2 = null; 
private DcMotor M3 = null; 
private DcMotor M4 = null; 
private DcMotor M5 = null;

@Override
public void runOpMode() {

M1 = hardwareMap.get(DcMotor.class, "M1"); 
M2 = hardwareMap.get(DcMotor.class, "M2"); 
M3 = hardwareMap.get(DcMotor.class, "M3"); 
M4 = hardwareMap.get(DcMotor.class, "M4"); 
M5 = hardwareMap.get(DcMotor.class, "M5"); 

M1.setPower(1); 
M3.setPower(1);
M2.setPower(1); 
M4.setPower(1); 
M5.setPower(1);

waitForStart();


while (opModeIsActive()) {

M1.setPower(-gamepad1.left_stick_y); 
M3.setPower(gamepad1.left_stick_y);
M2.setPower(-gamepad1.left_stick_y); 
M4.setPower(gamepad1.left_stick_y); 

M1.setPower(gamepad1.left_stick_x); 
M3.setPower(gamepad1.left_stick_x);
M2.setPower(gamepad1.left_stick_x); 
M4.setPower(gamepad1.left_stick_x); 

M1.setPower(gamepad1.left_stick_x); 
M3.setPower(gamepad1.left_stick_x);
M2.setPower(gamepad1.left_stick_x); 
M4.setPower(gamepad1.left_stick_x); 

M1.setPower(-gamepad1.right_stick_x); 
M3.setPower(gamepad1.right_stick_x);
M2.setPower(gamepad1.right_stick_x); 
M4.setPower(-gamepad1.right_stick_x); 

M5.setPower(gamepad1.right_stick_y);

if(gamepad1.a){
    M1.setPower(-gamepad1.left_stick_y * 0.5); 
    M3.setPower(gamepad1.left_stick_y * 0.5);
    M2.setPower(-gamepad1.left_stick_y * 0.5); 
    M4.setPower(gamepad1.left_stick_y * 0.5); 
}
}
}
}