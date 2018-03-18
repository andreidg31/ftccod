
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
/*
@Autonomous(name="Test Motoare", group="Linear Opmode")
//@Disabled
public class Test2 extends LinearOpMode {

    HardwareDORU            robot   = new HardwareDORU();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;



    public void setAll(double speed){
        robot.front_right_motor.setPower(speed);
        robot.front_left_motor.setPower(-speed);
        robot.back_right_motor.setPower(speed);
        robot.back_left_motor.setPower(-speed);
    }

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);
        waitForStart();

        robot.moveRobot(HardwareDORU.Directions.UP,0.6);
        sleep(1500);
        robot.stopRobot();

        robot.moveRobot(HardwareDORU.Directions.DOWN,0.6);
        sleep(1500);
        robot.stopRobot();

        robot.moveRobot(HardwareDORU.Directions.LEFT,0.6);
        sleep(1500);
        robot.stopRobot();

        robot.moveRobot(HardwareDORU.Directions.RIGHT,0.6);
        sleep(1500);
        robot.stopRobot();

        robot.moveRobot(HardwareDORU.Directions.LEFT_UP,0.6);
        sleep(1500);
        robot.stopRobot();

        robot.moveRobot(HardwareDORU.Directions.RIGHT_DOWN,0.6);
        sleep(1500);
        robot.stopRobot();

        robot.moveRobot(HardwareDORU.Directions.LEFT_DOWN,0.6);
        sleep(1500);
        robot.stopRobot();
        robot.moveRobot(HardwareDORU.Directions.RIGHT_UP,0.6);
        sleep(1500);
        robot.stopRobot();

        robot.turnRobot(HardwareDORU.Directions.LEFT,0.6);
        sleep(1000);
        robot.stopRobot();

        robot.turnRobot(HardwareDORU.Directions.RIGHT,0.6);
        sleep(1000);
        robot.stopRobot();
        stop();

    }
} */
