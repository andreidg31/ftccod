package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Cod Autonom", group="Linear Opmode")
//@Disabled
public class AutomatDORU extends LinearOpMode {

   // HardwareDORU            robot   = new HardwareDORU();
    HardwareFinalDORU robot = new HardwareFinalDORU();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
                                                       (WHEEL_DIAMETER_INCHES * 3.1415) * 2.54;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    //@Override

    public void setAll(double speed){
        robot.front_right_motor.setPower(speed);
        robot.front_left_motor.setPower(-speed);
        robot.back_right_motor.setPower(speed);
        robot.back_left_motor.setPower(-speed);
    }

    private  void driveRobot(double speed,
                             double CM,
                             int time){


        ElapsedTime tm = new ElapsedTime();
        int newRightTarget = robot.front_right_motor.getCurrentPosition() + (int)(CM* COUNTS_PER_CM);

        robot.front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.front_right_motor.setTargetPosition(newRightTarget);

        robot.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tm.reset();
        robot.front_left_motor.setPower(speed);
        robot.front_right_motor.setPower(speed);
        tm.startTime();

        while(tm.seconds()<time &&
              robot.front_right_motor.isBusy()){

                telemetry.addData("TargetRight",newRightTarget);
                telemetry.addData("CurrentRight",robot.front_right_motor.getCurrentPosition());
                telemetry.update();
        }
        robot.stopRobot();
    }

    private  void plaseazaCub(){

        robot.flip.setPosition(0.7);
        sleep(1500);
        robot.moveRobot(HardwareFinalDORU.Directions.BACKWARD,0.5);
        sleep(500);
        robot.flip.setPosition(0);
        sleep(1500);
    }

    int pozRelicva;
    public void runOpMode() throws InterruptedException {


       // robot.init(hardwareMap);
        //pozRelicva = robot.getPhoto(hardwareMap);
        robot.init(hardwareMap);
        pozRelicva   = robot.getPhoto(hardwareMap);

        telemetry.addData("Pozitie Relicva",pozRelicva);
        telemetry.update();
        waitForStart();

        robot.scula.setPosition(1);
        sleep(500);
        int red  = robot.color_sensor_scula.red();
        int blue = robot.color_sensor_scula.blue();
        if(red >blue){
            robot.turnRobot(HardwareFinalDORU.Directions.LEFT,0.6);
            sleep(200);
            robot.turnRobot(HardwareFinalDORU.Directions.RIGHT,0.6);
            sleep(200);
        }
        else{
            robot.turnRobot(HardwareFinalDORU.Directions.LEFT,0.6);
            sleep(200);
            robot.turnRobot(HardwareFinalDORU.Directions.RIGHT,0.6);
            sleep(200);
        }
        sleep(500);
        driveRobot(0.8,60.5,7);
        robot.stopRobot();
        sleep(500);
        robot.turnGyro(HardwareFinalDORU.Directions.LEFT,90,0.6);
        sleep(2000);
        robot.stopRobot();;
        if(pozRelicva == 1){
            robot.moveRobot(HardwareFinalDORU.Directions.LEFT,0.6);
            sleep(300);
        }
        else if(pozRelicva == 3){
            robot.moveRobot(HardwareFinalDORU.Directions.RIGHT,0.6);
            sleep(300);
        }
        robot.stopRobot();
        robot.moveRobot(HardwareFinalDORU.Directions.FORWARD,0.5);
        sleep(500);
        robot.stopRobot();
        plaseazaCub();
        idle();
    }
}
