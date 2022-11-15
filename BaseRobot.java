package org.firstinspires.ftc.teamcode;

//import android.util.Log;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.Map;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*

                DOCUMENT OF THE PORTS
                   as of 10/24/2022

            //Hubs
            Control Hub   = Control Hub
            Expansion Hub = Expansion Hub 2

            //Wheels
            frontLeft    --- 1, Control Hub
            frontRight   --- 0, Control Hub
            backLeft     --- 1, Expansion Hub
            backRight    --- 0, Expansion Hub

            //Arm/Slide
            arm          --- 2, Expansion Hub
            pulley       --- 2, Control Hub



            //Servos
            grip       --- 0, Control Hub
                -------------------------

                    SIGNATURES
            Alan is dumb  Dallin Waite
            Samuel Childers  Dexter

*/

public class BaseRobot {


    //Initialize Variables
    HardwareMap hwMap = null;

    //DistanceSensor distance;
    //DcMotor spin;
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    BNO055IMU imu;

    boolean go = true;
    int leftPos;
    int rightPos;
    int currentTicks;
    int barcode = 0;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    public double subtractor = 0;
    private double deltaAngle = 0;
    private double defaultPower = 0.25;
    private double savedAngle = 0;
    private double error = 0;


    //FUNCTIONS
    public void init(HardwareMap ahwMap) {
        //Get hardware Map
        hwMap = ahwMap;

        //Initialize gyro sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Unbreaks something with gyro yes ask dallin he will explain for a long time
        // it just makes sure that the gyro gets calibrated. simple, alan
        //its not, just fun to make yuou  mad (;
        while (!imu.isGyroCalibrated()) {

        }

        // Get the distance sensor and motor from hardwareMap
        //distance = hwMap.get(DistanceSensor.class, "distance");
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
    }
    //Save gyro position
    public void saveGyro() {
        savedAngle = -this.getAngle();
    }

    public void resetToSaved() {
        this.turn(this.getAngle() - savedAngle);
    }

    //driving methods
    public void drive(int target, double fl, double bl, double fr, double br){
        this.saveGyro();

        //Drive straight using encoders
        currentTicks = 0;
        //Reset and activate encoders for the back left motor
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Run this until we hit the target encoder ticks
        while (currentTicks < target) {
            //Update tick value
            if (bl < 0) {
                currentTicks = -backLeft.getCurrentPosition();
            } else {
                currentTicks = backLeft.getCurrentPosition();
            }

            //calculate angle from original
            error = -this.getAngle() - savedAngle;

            this.correctError(fl, bl, fr, br);
        }
        //Once we've hit the target ticks, stop motors
        this.stop();
        this.resetToSaved();
    }
    //correct from error
    public void correctError(double pwr) {
        if (error > 0.5) {
            this.setPower(pwr-0.05, pwr-0.05, pwr+0.05, pwr+0.05);
        } else if (error < -0.5) {
            this.setPower(pwr+0.05, pwr+0.05, pwr-0.05, pwr-0.05);
        } else {
            //Go
            this.setPower(pwr);
        }
    }

    public void correctError(double fl, double bl, double fr, double br) {
        if (error > 0.5) {
            this.setPower(fl-0.05, bl-0.05, fr+0.05, br+0.05);
        } else if (error < -0.5) {
            this.setPower(fl+0.05, bl+0.05, fr-0.05, br-0.05);
        } else {
            //Go
            this.setPower(fl, bl, fr, br);
        }
    }

    //Gyro reset to last saved position

    //forwards
    public void forward(int target) {
        drive(target, defaultPower, defaultPower, defaultPower, defaultPower);
    }
    public void forward(int target, double pwr) {
        drive(target, pwr, pwr, pwr, pwr);
    }
    //backwards
    public void back(int target) {
        drive(target, -defaultPower, -defaultPower, -defaultPower, -defaultPower);
    }
    public void back(int target, double pwr) {
        drive(target, -pwr, -pwr, -pwr, -pwr);
    }
    //STRAFING
    //strafe left
    public void strafeLeft(int target) {
        drive(target, -defaultPower, defaultPower, defaultPower, -defaultPower);
    }
    public void strafeLeft(int target, double pwr) {
        drive(target, -pwr, pwr, pwr, -pwr);
    }
    //strafe right
    public void strafeRight(int target) {
        drive(target, defaultPower, -defaultPower, -defaultPower, defaultPower);
    }
    public void strafeRight(int target, double pwr) {
        drive(target, pwr, -pwr, -pwr, pwr);
    }

    //Original Stuff
    //same as above, but backwards
    public void backOg(int target) {
        int currentTicks = 0;
        this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (currentTicks < target) {
            currentTicks = -backLeft.getCurrentPosition();
            this.setPower(-0.25);
        }
        this.stop();
    }

    public void turnRightOg(int target) {
        target = (int) java.lang.Math.round(target * 12.889);
        currentTicks = 0;
        target = target * -1;
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (currentTicks > target) {
            currentTicks = -backLeft.getCurrentPosition();
            frontLeft.setPower(0.4);
            frontRight.setPower(-0.4);
            backLeft.setPower(0.4);
            backRight.setPower(-0.4);
        }
        this.stop();
    }

    public void turnLeftOg(int target) {
        target = (int) java.lang.Math.round(target * 12.889);
        currentTicks = 0;
        double lpowerLeft = 0.4;
        double lpowerRight = 0.4;
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (currentTicks < target) {
            currentTicks = -backLeft.getCurrentPosition();
            frontLeft.setPower(-lpowerLeft);
            frontRight.setPower(lpowerRight);
            backLeft.setPower(-lpowerLeft);
            backRight.setPower(lpowerRight);
        }
        this.stop();
    }

    public void stop() {
        //set motor power to 0
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
        this.backLeft.setPower(0);
        this.backRight.setPower(0);
        //this.wand.setPower(0);
        //this.flapper.setPower(0);
    }

    public void setPower(double pwr) {
        //set motor power to user input
        this.frontLeft.setPower(pwr);
        this.backLeft.setPower(pwr);
        this.frontRight.setPower(pwr);
        this.backRight.setPower(pwr);
    }

    public void setPower(double fl, double bl, double fr, double br) {
        //set motor power to user input
        this.frontLeft.setPower(fl);
        this.backLeft.setPower(bl);
        this.frontRight.setPower(fr);
        this.backRight.setPower(br);
    }

    public void resetAngle() {
        //Set angle to 0
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        subtractor = orientation.firstAngle;
    }

    public double getAngle() {
        //get our current angle
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        deltaAngle = orientation.firstAngle;
        deltaAngle -= subtractor;

        if (deltaAngle > 190) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -190) {
            deltaAngle += 360;
        }
        return deltaAngle;
    }

    public int straight(double target) {
        this.resetAngle();
        double angle = -this.getAngle();
        //Drive straight using encoders
        int currentTicks = 0;
        double powerLeft = 0.5;
        double powerRight = 0.5;
        this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Drive until encoder count reaches target
        while (currentTicks < target) {
            angle = -this.getAngle();
            if (angle > 0.5) {
                powerLeft = 0.35;
                powerRight = 0.65;
            } else if (angle < -0.5) {
                powerLeft = 0.65;
                powerRight = 0.35;
            } else {
                powerLeft = 0.5;
                powerRight = 0.5;
            }
            currentTicks = frontRight.getCurrentPosition();
            frontLeft.setPower(powerLeft);
            frontRight.setPower(powerRight);
            backLeft.setPower(powerLeft);
            backRight.setPower(powerRight);
        }
        this.stop();
        return currentTicks;
    }

    public void turn(double target) {
        //Turn desired number of degrees
        this.resetAngle();
        double angle = -this.getAngle();
        //check if turning right
        if (target > 0) {
            while (angle < target) {
                //turn till correct angle
                angle = -this.getAngle();
                this.setPower(0.17, 0.17, -0.17, -0.17);
            }
            this.stop();
            while (angle > target) {
                //correct slowly
                angle = -this.getAngle();
                this.setPower(-0.1, -0.1, 0.1, 0.1);
            }
            this.stop();
        } else {
            //if turning left
            while (angle > target) {
                //turn till correct angle
                angle = -this.getAngle();
                this.setPower(-0.17, -0.17, 0.17, 0.17);
            }
            this.stop();
            while (angle < target) {
                //correct slowly
                angle = -this.getAngle();
                this.setPower(0.1, 0.1, -0.1, -0.1);
            }
            this.stop();
        }
    }
}
