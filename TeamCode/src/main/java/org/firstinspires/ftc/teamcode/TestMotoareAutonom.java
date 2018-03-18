package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestMotoareAutonom", group="Linear Opmode")
//@Disabled
public class TestMotoareAutonom extends LinearOpMode {

    HardwareDORU            robot   = new HardwareDORU();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
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

    private  void driveRobot(int speed,
                             int CM,
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

    int pozRelicva;
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);
        //pozRelicva = robot.getPhoto(hardwareMap);
        telemetry.addData("Pozitie Relicva",pozRelicva);

        waitForStart();

       // robot.scula.setPosition(1);
        robot.moveRobot(HardwareDORU.Directions.FORWARD,0.6);
        sleep(2000);
        robot.stopRobot();
        sleep(500);

        robot.moveRobot(HardwareDORU.Directions.BACKWARD,0.6);
        sleep(2000);
        robot.stopRobot();
        sleep(500);
        robot.moveRobot(HardwareDORU.Directions.LEFT,0.6);
        sleep(2000);
        robot.stopRobot();
        sleep(500);
        robot.moveRobot(HardwareDORU.Directions.RIGHT,0.6);
        sleep(2000);
        robot.stopRobot();
        sleep(500);
        robot.moveRobot(HardwareDORU.Directions.LEFT_FORWARD,0.6);
        sleep(2000);
        robot.stopRobot();
        sleep(500);
        robot.moveRobot(HardwareDORU.Directions.RIGHT_BACKWARD,0.6);
        sleep(2000);
        robot.stopRobot();
        sleep(500);
        robot.moveRobot(HardwareDORU.Directions.RIGHT_FORWARD,0.6);
        sleep(2000);
        robot.stopRobot();
        sleep(500);
        robot.moveRobot(HardwareDORU.Directions.LEFT_BACKWARD,0.6);
        sleep(2000);
        robot.stopRobot();
        sleep(500);
    }
}
