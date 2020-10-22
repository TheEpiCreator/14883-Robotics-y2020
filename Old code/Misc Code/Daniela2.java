package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Daniela2 (Blocks to Java)", group = "")
public class Daniela2 extends LinearOpMode {

  private DcMotor left_drive;
  private DcMotor right_drive;
  private DcMotor intake_drive;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    left_drive = hardwareMap.dcMotor.get("left_drive");
    right_drive = hardwareMap.dcMotor.get("right_drive");
    intake_drive = hardwareMap.dcMotor.get("intake_drive");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      float leftPower = 0;
      float rightPower = 0;
      float servoPower = 0;
      while (opModeIsActive()) {
        // Put loop blocks here.
        leftPower = gamepad1.right_stick_y-(gamepad1.right_stick_x/4);
        rightPower = gamepad1.right_stick_y+(gamepad1.right_stick_x/4);
        telemetry.addData("Tel", "Tel");
        if(gamepad1.a == true){
          servoPower = 50;
        }else if(gamepad1.b == true){
          servoPower = -50;
        }
        
        
        if(gamepad1.right_bumper == true){
          leftPower = 5;
          rightPower = -5;
        }else if(gamepad1.left_bumper == true){
          leftPower = -5;
          rightPower = 5;
        }else if(gamepad1.right_bumper == true && gamepad1.left_bumper == true){
          leftPower = 10;
          rightPower = 10;
        }
        
        
        left_drive.setPower(leftPower);
        right_drive.setPower(rightPower);
        intake_drive.setPower(servoPower);
        telemetry.update();
      }
  }
}}
