package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; 

@TeleOp(name="Basic_Drive_Train", group="Concept") 
public class BasicDriveTrain extends LinearOpMode {

private DcMotor M1 = null; 
private DcMotor M2 = null; 
private DcMotor M3 = null; 
private DcMotor M4 = null; 

@Override
public void runOpMode() {

M1 = hardwareMap.get(DcMotor.class, "M1"); 
M2 = hardwareMap.get(DcMotor.class, "M2"); 
M3 = hardwareMap.get(DcMotor.class, "M3"); 
M4 = hardwareMap.get(DcMotor.class, "M4"); 

M1.setPower(0.75); 
M3.setPower(0.75);
M2.setPower(0.75); 
M4.setPower(0.75); 


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

if(gamepad1.a){
    M1.setPower(-gamepad1.left_stick_y * 0.66); 
    M3.setPower(gamepad1.left_stick_y * 0.66);
    M2.setPower(-gamepad1.left_stick_y * 0.66); 
    M4.setPower(gamepad1.left_stick_y * 0.66); 

}

if(gamepad1.b){
    M1.setPower(-gamepad1.left_stick_y * 1.33); 
    M3.setPower(gamepad1.left_stick_y * 1.33);
    M2.setPower(-gamepad1.left_stick_y * 1.33); 
    M4.setPower(gamepad1.left_stick_y * 1.33); 
} 
}
}
}