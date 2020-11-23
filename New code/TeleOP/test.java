package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Final Bot", group = "control")

public class MecanumDaniel_Unlaggy extends LinearOpMode {

    // Motor
    private DcMotor test = null;
    private double[] motorValues = {0, 0, 0, 0};
    private double[] motorRotation = {0, 0, 0, 0};
        
    // Controller input
    private double leftStickX = 0;
    private boolean[] ButtonStates = {false};
    

    public void runOpMode() throws InterruptedException {
    
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        test = hardwareMap.get(DcMotor.class, "test");
        
        
        test.setDirection(DcMotor.Direction.FORWARD);

        
        test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        
        waitForStart();

        while (opModeIsActive()) {
            
            leftStickX = gamepad1.left_stick_x;
            
            test.setPower(leftStickX * 5);
            
        }
    }
}