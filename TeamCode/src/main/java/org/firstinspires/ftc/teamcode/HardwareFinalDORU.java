package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

//import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
//import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRGyro;

public class HardwareFinalDORU
{
    public enum Directions{
        LEFT,RIGHT,FORWARD,BACKWARD,
        LEFT_FORWARD,RIGHT_FORWARD,LEFT_BACKWARD,RIGHT_BACKWARD;
    }
    /* Public OpMode members. */
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    public DcMotor  front_left_motor  = null;
    public DcMotor  front_right_motor = null;
    public DcMotor  back_left_motor   = null;
    public DcMotor  back_right_motor  = null;
    public DcMotor  rotorStanga       = null;
    public DcMotor  rotorDreapta      = null;
    public DcMotor  elevator_motor    = null;
    public DcMotor  motorExtender     = null;

    public Servo flip              = null;
    public Servo scula             = null; // Pentru ca se scoala si se culca
    public Servo bascula           = null;
    public Servo relicClaw         = null;
    public Servo relicArm          = null;
   // public Servo    left_claw         = null;
   // public Servo    right_claw        = null;

    public ColorSensor color_sensor_scula = null;
    public ColorSensor color_sensor_under = null;
    public GyroSensor  gyro               = null;
   // public Servo    lateral_servo     = null;

    public HardwareFinalDORU(){

    }

   public int getPhoto(HardwareMap hw) {

        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQWGJ7j/////AAAAmYijk0JarkHXh/1DC8/mwvZJvuQxzZEC+kLP/PQ/gSPPJztZKb46n3QsAHyg5kGjwxDLu11Ino+7GxmUSUkJVJEme80x15p3kLM/x4GscpCCdwOh6R+C0qbl7fDasjptQjg1+T/8G6646PSuC0jB6RteexdDbRGkJxZNWbxk0ZvR5Qtcjp8r0zi1LLi6toi0P6lWAYzPQyxn0OGMESddFLmnnjkwoMX1u5D9Elqqi7BqqJNjTE5goTvxnneWUCpX0ly7Fsu/nHftiD6A1wbqQR4fY2YlrHjWFcv0scMzSZUTNgeIDCHEbunmfGgGSkeGqXJ8+z64Nv4NZ9SmNb3auE9BmGsJpQKUjjmN6eegJXJw";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        ElapsedTime tm = new ElapsedTime();
        tm.reset();
        tm.startTime();

       //Cat timp verifica relicva
       //Modifica cifra pentru a il pune sa verifice un timp mai scurt
        while(tm.seconds()<5) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return 1; // Poza spre stanga
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return 2; // Poza spre centru
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return 3; // Poza spre dreapta
                }
//                if (pose != null) {
//                    VectorF trans = pose.getTranslation();
//                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
//                    double tX = trans.get(0);
//                    double tY = trans.get(1);
//                    double tZ = trans.get(2);
//
//                    // Extract the rotational components of the target relative to the robot
//                    double rX = rot.firstAngle;
//                    double rY = rot.secondAngle;
//                    double rZ = rot.thirdAngle;
//                   // DcMotor g = new DcMotor();
//                   // g.getPower()
            }
        }
       return 0;
    }

    public void init(HardwareMap hw) {

        front_left_motor  = hw.dcMotor.get("front_left_motor");
        front_right_motor = hw.dcMotor.get("front_right_motor");
        back_left_motor   = hw.dcMotor.get("back_left_motor");
        back_right_motor  = hw.dcMotor.get("back_right_motor");
        rotorStanga       = hw.dcMotor.get("rotorStanga");
        rotorDreapta      = hw.dcMotor.get("rotorDreapta");
        motorExtender     = hw.dcMotor.get("motorExtender");
        gyro              = hw.gyroSensor.get("gyro");
        elevator_motor    = hw.dcMotor.get("elevator_motor");
        flip              = hw.servo.get("flip");
       // push_cube         = hw.servo.get("push_cube");
        bascula           = hw.servo.get("bascula");
        scula             = hw.servo.get("scula");

        color_sensor_scula = hw.colorSensor.get("color_sensor_scula");
        color_sensor_under = hw.colorSensor.get("color_sensor_under");

        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotorStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotorDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        flip.setPosition(0.05);
        bascula.setPosition(0);
        scula.setPosition(0);
        relicArm.setPosition(0);
        relicClaw.setPosition(1);

        color_sensor_scula.enableLed(false);
        color_sensor_under.enableLed(false);

        //front_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //back_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        calibrateGyro();
    }

    void turnRobot(Directions dir,double Power) {


        if(dir == Directions.LEFT){

            front_left_motor.setPower(-Power);
            back_left_motor.setPower(-Power);
            front_right_motor.setPower(Power);
            back_right_motor.setPower(Power);
        }
        else if(dir == Directions.RIGHT){

            front_left_motor.setPower(Power);
            back_left_motor.setPower(Power);
            front_right_motor.setPower(-Power);
            back_right_motor.setPower(-Power);
        }
    }

    void turnGyro(Directions dir, double angle,double Power) {


        if(dir == Directions.LEFT){


            double anglegyro = (360 * gyro.getHeading() + 360 + angle)%360;
            if(anglegyro==0)
                anglegyro = 360;

            while(gyro.getHeading()<anglegyro/360){
                front_left_motor.setPower(-Power);
                back_left_motor.setPower(-Power);
                front_right_motor.setPower(Power);
                back_right_motor.setPower(Power);
            }
            stopRobot();
        }
        else if(dir == Directions.RIGHT){



            double anglegyro = (360 * gyro.getHeading() + 360 + angle)%360;
            if(anglegyro==0)
                anglegyro = 360;

            while(gyro.getHeading()>anglegyro/360){
                front_left_motor.setPower(Power);
                back_left_motor.setPower(Power);
                front_right_motor.setPower(-Power);
                back_right_motor.setPower(-Power);
            }
            stopRobot();
        }
    }

    void  moveRobot(Directions dir,double Power ){

        front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        if(dir == Directions.FORWARD){

            front_left_motor.setPower(Power);
            front_right_motor.setPower(Power);
            back_right_motor.setPower(Power);
            back_left_motor.setPower(Power);
        }
        else if(dir == Directions.BACKWARD){

            front_left_motor.setPower(-Power);
            front_right_motor.setPower(-Power);
            back_right_motor.setPower(-Power);
            back_left_motor.setPower(-Power);
        }
        else if(dir == Directions.LEFT){

            front_left_motor.setPower(-Power);
            front_right_motor.setPower(Power);
            back_right_motor.setPower(-Power);
            back_left_motor.setPower(Power);
        }
        else if(dir == Directions.RIGHT){

            front_left_motor.setPower(Power);
            front_right_motor.setPower(-Power);
            back_right_motor.setPower(Power);
            back_left_motor.setPower(-Power);
        }
        else if(dir == Directions.LEFT_BACKWARD){

            front_left_motor.setPower(-Power);
            front_right_motor.setPower(0);
            back_right_motor.setPower(-Power);
            back_left_motor.setPower(0);
        }
        else if(dir == Directions.LEFT_FORWARD){

                front_left_motor.setPower(0);
                front_right_motor.setPower(Power);
                back_right_motor.setPower(0);
                back_left_motor.setPower(Power);
        }
        else if(dir == Directions.RIGHT_BACKWARD){

            front_left_motor.setPower(0);
            front_right_motor.setPower(-Power);
            back_right_motor.setPower(0);
            back_left_motor.setPower(-Power);
        }
        else if(dir == Directions.RIGHT_FORWARD){

            front_left_motor.setPower(Power);
            front_right_motor.setPower(0);
            back_right_motor.setPower(Power);
            back_left_motor.setPower(0);
        }
    }


    void stopRobot(){

        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_right_motor.setPower(0);
        back_left_motor.setPower(0);
    }

    void calibrateGyro(){

        gyro.calibrate();
        while(gyro.isCalibrating()){

        }
        gyro.resetZAxisIntegrator();

    }

}


