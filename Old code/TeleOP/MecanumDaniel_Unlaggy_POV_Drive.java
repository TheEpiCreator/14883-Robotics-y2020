package daniel;

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

@TeleOp(name = "Final Bot POV Drive", group = "control")

public class MecanumDaniel_Unlaggy_POV_Drive extends LinearOpMode {

    private DcMotor lF = null;
    private DcMotor rF = null;
    private DcMotor lB = null;
    private DcMotor rB = null;
    
    private Servo autoServo = null;
    private Servo autoServoLeft = null;
    private double servoRot = -0.4;
    
    private boolean servoState = false;
    private boolean driveState = true;
    
    private double servoRotA = 0;
    private double servoRotB = 0;
    private double timer = 0;
    private double offset = 0;
    
    private boolean correctionEnabled = false;
    private double deadzone = 0.04;
    
    private double leftStickX = 0;
    private double leftStickY = 0;
    private double rightStickX = 0;

    private boolean buttonA;
    private boolean buttonB;
    private boolean buttonX;
    
    private double[] motorValues = {0, 0, 0, 0};
    private double[] motorRotation = {0, 0, 0, 0};
    
    private double correctionFinal;
    
    private boolean[] ButtonStates = {false, false};

    private double joyIntensity;
    private double joyAngle;
    private double xAngleOut;
    private double yAngleOut;
    
    //TouchSensor touch
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
        
                
        //wait until gyro calibrated
        while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }
        
        telemetry.addData("imu calibration", imu.getCalibrationStatus().toString());
        telemetry.update();
        
        waitForStart();
        
        sleep(1000);

        while (opModeIsActive()) {
            
            correction = checkDirection();
            
            leftStickY = -gamepad1.left_stick_y;
            leftStickX = gamepad1.left_stick_x;
            rightStickX = gamepad1.right_stick_x;
            buttonA = gamepad1.a;
            buttonB = gamepad1.b;
            buttonX = gamepad1.x;
            
            offset -= rightStickX;
            
            correctionFinal = correction*0.01 - offset;
            
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
            

            //toggle POV drive
            if(buttonX && ButtonStates[1] == false){
                if(servoState){
                    driveState = false;
                }else{
                    driveState = true;
                }
            }
            if(buttonX){
                ButtonStates[1] = true;
            } else {
                ButtonStates[1] = false;
            }


            //use IMU gyro correction to offset controls and achieve POV drive
            //only activate POV drive if needed
            if(driveState){
                //filter & apply joystick
                leftStickX = calculateAngleFilter(leftStickX, leftStickY, correction, false);
                leftStickY = calculateAngleFilter(leftStickX, leftStickY, correction, false);
            }

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
                mecDrive(leftStickY,rightStickX,leftStickX);
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
            servoRotB = servoRot+0.5;
            autoServo.setPosition(servoRotA);
            autoServoLeft.setPosition(servoRotB);
            timer++;
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

    private double[] calculateAngleFilter(double xAngleIn, double yAngleIn, double rAngleIn, boolean outputMode) {
        joyIntensity = Math.sqrt(xAngleIn*xAngleIn + yAngleIn*yAngleIn);
        joyAngle = Math.atan2(yAngleIn, xAngleIn);
        xAngleOut = Math.cos(joyAngle+rAngleIn)*joyIntensity;
        yAngleOut = Math.sin(joyAngle+rAngleIn)*joyIntensity;
	if(outputMode == false){
            return xAngleOut;
	}else{
	    return yAngleOut;
	}
    }


    }
    
