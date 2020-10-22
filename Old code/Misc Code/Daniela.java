package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Daniela (Blocks to Java)", group = "")
public class Daniela extends LinearOpMode {

  private DcMotor left_top_drive;
  private DcMotor right_top_drive;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    left_top_drive = hardwareMap.dcMotor.get("left_top_drive");
    right_top_drive = hardwareMap.dcMotor.get("right_top_drive");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        left_top_drive.setPower(gamepad1.left_stick_x - gamepad1.left_stick_y);
        right_top_drive.setPower(gamepad1.left_stick_x + gamepad1.left_stick_y);
        telemetry.update();
      }
    }
  }
}
