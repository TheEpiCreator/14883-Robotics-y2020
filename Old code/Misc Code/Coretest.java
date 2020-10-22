package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.io.Console;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Core Hex Motor Test", group = "driving")
public class Coretest extends LinearOpMode {

  private DcMotor test;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    test = hardwareMap.get(DcMotor.class, "test");


    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        
        //values for wheels:
        //forward: 1, 1, 1, 1
        //backward: -1, -1, -1, -1
        //left: -1, 1, 1, -1
        //right: 1, -1, -1, 1
        //gamepad.
        //calculate wheel power
        test.setPower(1);
        telemetry.update();
      }
    }
  }
}
