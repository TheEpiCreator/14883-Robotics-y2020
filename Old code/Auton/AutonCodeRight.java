package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "Autonomous Final Move Right", group = "Autonomous")

public class AutonCodeRight extends LinearOpMode {
    private DcMotor lF = null;
    private DcMotor rF = null;
    private DcMotor lB = null;
    private DcMotor rB = null;
    
    private Servo autoServo = null;
    private Servo autoServoLeft = null;
    private double servoRot = -0.4;
    
    private boolean servoState = true;
    
    private double servoRotA = 0;
    private double servoRotB = 0;
    private double offset = 0;
    
    private boolean correctionEnabled = false;
    private double deadzone = 0.04;
    //controller    
    private double leftStickX = 0;
    private double leftStickY = 0;
    private double rightStickX = 0;

    private boolean buttonA = false;
    private boolean buttonB = false;
    //drive
    private double[] motorValues = {0, 0, 0, 0};
    private double[] motorRotation = {0, 0, 0, 0};
    //gyro
    private double correctionFinal;
    
    //servo controls
    private boolean[] ButtonStates = {false};
    
    //time
    private double autoTimer = 30;
    private int timerStage = 0;
    
    //time
    private ElapsedTime     runtime = new ElapsedTime();
    private double offsetTime = 0;

    
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    private double globalAngle, power = .30, correction;
    boolean aButton, bButton, touched;

    public void runOpMode() throws InterruptedException {
    
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        
        lF = hardwareMap.get(DcMotor.class, "lF");
        rF = hardwareMap.get(DcMotor.class, "rF");
        lB = hardwareMap.get(DcMotor.class, "lB");
        rB = hardwareMap.get(DcMotor.class, "rB");
        
        
        autoServo = hardwareMap.get(Servo.class, "autoservo");
        autoServoLeft = hardwareMap.get(Servo.class, "autoservoleft");
        
        //touch = hardwareMap.get(TouchSensor.class, "touch");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        imu.initialize(parameters);
        
        lF.setDirection(DcMotor.Direction.FORWARD);
        rF.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.FORWARD);
        rB.setDirection(DcMotor.Direction.REVERSE);
        
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        /*lF.setTargetPosition(0);
        rF.setTargetPosition(0);
        lB.setTargetPosition(0);
        rB.setTargetPosition(0);

        
        lF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rB.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        
        //wait until gyro calibrated
        while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }
        
        telemetry.addData("imu calibration", imu.getCalibrationStatus().toString());
        telemetry.update();
        
        waitForStart();
        
        sleep(1000);

        offsetTime = runtime.seconds();

        while (opModeIsActive()) {
            
            correction = checkDirection();
            
            autoTimer = 30-runtime.seconds() + offsetTime;
            
            leftStickY = 0;
            leftStickX = 0;
            rightStickX = 0;
            buttonA = false;
            buttonB = false;

            if(autoTimer>28.5){
                leftStickX = 0.5;
            }
            telemetry.addData("Time left", autoTimer);
            offset -= rightStickX;
            
            correctionFinal = correction*0.01 - offset;
            telemetry.addData("correction with offset", correctionFinal);
            
            //gyro correction deadzone
            if(correction < -deadzone){
                lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                correctionFinal -= deadzone;
                
                lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }else if(correction > deadzone){
                correctionFinal += deadzone;
            }else if(Math.abs(correction) < deadzone){
                correctionFinal = 0;
            }
            
            telemetry.addData("adjusted correction", correctionFinal);
            
            if(correctionEnabled == true){
                lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                
                mecDrive(leftStickY,-correctionFinal,leftStickX);
                lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                
            }else{
                mecDrive(leftStickY,-rightStickX,leftStickX);
            }
            
            //servos
            //switch states
            if(buttonA && ButtonStates[0] == false){
                if(servoState){
                    servoState = false;
                }else{
                    servoState = true;
                }
            }
            
            if(buttonA){
                ButtonStates[0] = true;
            } else {
                ButtonStates[0] = false;
            }
            
            //switch servo based on states
            if(servoState){
                servoRot = -0.6;
            } else {
                servoRot = -0.05;
            }
            
            if(buttonB){
                servoRot = 0;
            }
            servoRotA = -servoRot+0.5;
            telemetry.addData("servoRotA", servoRotA);
            servoRotB = servoRot+0.5;
            telemetry.addData("servoRotB", servoRotB);
            telemetry.addData("servoRot", servoRot);
            autoServo.setPosition(servoRotA);
            autoServoLeft.setPosition(servoRotB);
            
            telemetry.addData("angle", getAngle());
            telemetry.update();
            
        }
    }
    public void mecDrive(double forward, double turn, double strafe) {
        
        motorValues[0] = forward - turn + strafe;
        motorValues[1] = forward + turn - strafe;
        motorValues[2] = forward - turn - strafe;
        motorValues[3] = forward + turn + strafe;
        
        leftStickX *= 4;
        leftStickY *= 4;
        rightStickX *= 4;
        
        leftStickX = Math.pow(leftStickX, 5);
        leftStickY = Math.pow(leftStickY, 5);
        rightStickX = Math.pow(rightStickX, 5);
        
        for(int i=0; i<motorValues.length; i++){
            //motorValues[i] /= 1.8;
            //motorValues[i] = Math.pow(motorValues[i], 3);
            
            
            //motorRotation[i] += motorValues[i] * 100;
        }
        
        lF.setPower(motorValues[0]);
        rF.setPower(motorValues[1]);
        lB.setPower(motorValues[2]);
        rB.setPower(motorValues[3]);
        
        /*lF.setTargetPosition((int) motorRotation[0]);
        rF.setTargetPosition((int) motorRotation[1]);
        lB.setTargetPosition((int) motorRotation[2]);
        rB.setTargetPosition((int) motorRotation[3]);*/
        
    }
    
        private double getAngle(){
        
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        
        if(deltaAngle < -180){
            deltaAngle += 360;
        }else if (deltaAngle > 180){
            deltaAngle -= 360;
        }
        
        globalAngle += deltaAngle;
        lastAngles = angles;
        
        return globalAngle;
    }
    
    private double checkDirection(){
        double correction, angle, gain = 1;
        
        
        angle = getAngle();
        
        if(angle == 0){
            correction = 0;
        }else{
            correction = -angle;
        }
        
        correction = correction*gain;
        
        return correction;
        
    }

    }
    
