/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Cod Controlat6", group="Linear Opmode")
public class ControlatDORU extends LinearOpMode {

    HardwareDORU robot = new HardwareDORU();
    public boolean isOpen = false;
    public boolean isRunning = false;
    public boolean isFlipped = false;
    public boolean isBasculat = false;
    public  ElapsedTime time = new ElapsedTime();
    public  ElapsedTime time_flip = new ElapsedTime();

    public  ElapsedTime time_bascula = new ElapsedTime();
    public void Drive() {

//        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
//        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
//        double rightX = gamepad1.right_stick_x;
//        final double v1 = r * Math.cos(robotAngle) - rightX;
//        final double v2 = r * Math.sin(robotAngle) + rightX;
//        final double v3 = r * Math.sin(robotAngle) - rightX;
//        final double v4 = r * Math.cos(robotAngle) + rightX;
//
//        robot.front_left_motor.setPower(-v1);
//        robot.front_right_motor.setPower(v2);
//        robot.back_left_motor.setPower(-v3);
//        robot.back_right_motor.setPower(v4);

        double r = Math.hypot(-gamepad1.right_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        robot.front_left_motor.setPower(-v1);
        robot.front_right_motor.setPower(-v2);
        robot.back_left_motor.setPower(-v3);
        robot.back_right_motor.setPower(-v4);
    }

    public void DriveStanga() {
        if(gamepad1.x) {
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle + Math.PI/2) - rightX;
            final double v2 = r * Math.sin(robotAngle + Math.PI/2) + rightX;
            final double v3 = r * Math.sin(robotAngle + Math.PI/2) - rightX;
            final double v4 = r * Math.cos(robotAngle + Math.PI/2) + rightX;

            robot.front_left_motor.setPower(-v1);
            robot.front_right_motor.setPower(v2);
            robot.back_left_motor.setPower(-v3);
            robot.back_right_motor.setPower(v4);
        }
    }
//    }
//    public void rotor(){
//        if(time.seconds()>1) {
//            time.reset();
//            if (gamepad1.a) {
//
//                if (isRunning == false) {
//                    robot.rotorDreapta.setPower(1);
//                    robot.rotorStanga.setPower(-1);
//                    isRunning = true;
//                } else {
//                    robot.rotorDreapta.setPower(0);
//                    robot.rotorStanga.setPower(0);
//                    isRunning = false;
//                }
//            } else if (gamepad1.b) {
//                robot.rotorDreapta.setPower(-1);
//                robot.rotorStanga.setPower(1);
//            }
//        }
//    }
//
//    public void rotor_b(){
//        if(gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0){
//            if(gamepad1.right_trigger > 0) {
//                robot.rotorDreapta.setPower(-1);
//                robot.rotorStanga.setPower(1);
//            }
//            else{
//                robot.rotorDreapta.setPower(1);
//                robot.rotorStanga.setPower(-1);
//            }
//        }
//        else{
//            robot.rotorDreapta.setPower(0);
//            robot.rotorStanga.setPower(0);
//        }
//    }
  /*  public void Catch(){
        if(gamepad1.a) {
            if (isOpen == false) {
                robot.left_claw.setPosition(0.5);
                robot.right_claw.setPosition(0.4);
                sleep(250);
                isOpen = true;
            } else {
                robot.left_claw.setPosition(0.65);
                robot.right_claw.setPosition(0.25);
                sleep(250);
                isOpen = false;
            }
        }
        //sleep(500);
    }
*/
//    public void Elevator(){
//        if(gamepad1.dpad_down || gamepad1.dpad_up){
//            if(gamepad1.dpad_up)
//                robot.elevator_motor.setPower(-0.8);
//            if(gamepad1.dpad_down)
//                robot.elevator_motor.setPower(0.1);
//        }
//        else
//            robot.elevator_motor.setPower(0);
//    }
//
//    public void Flip(){
//        if(time_flip.seconds() > 1){
//            if(gamepad1.right_bumper || gamepad1.left_bumper || gamepad1.y){
//                time_flip.reset();
//                if(gamepad1.y){
//                    robot.flip.setPosition(0.0);
//                    return;
//                }
//                if(gamepad1.right_bumper){
//                        robot.flip.setPosition(0.06);
//                    return;
//                }
//                if(gamepad1.left_bumper){
//                    robot.flip.setPosition(0.05);
//                    return;
//                }
//
//            }
//
//        }
//    }
//
//
//    public void bascula(){
//        if(time_bascula.seconds() > 1){
//            if(gamepad1.x){
//                time_bascula.reset();
//                if(isBasculat == false){
//                    robot.bascula.setPosition(0.8);
//                    isBasculat = true;
//                }
//                else{
//                    robot.bascula.setPosition(0);
//                    isBasculat = false;
//                }
//            }
//        }
//    }
//
//
//   // @Override


    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData("ConnectionInfo1",robot.front_left_motor.getConnectionInfo());
        telemetry.addData("ConnectionInfo2",robot.front_right_motor.getConnectionInfo());
        telemetry.addData("ConnectionInfo3",robot.back_left_motor.getConnectionInfo());
        telemetry.addData("ConnectionInfo4",robot.back_right_motor.getConnectionInfo());
        telemetry.update();
        sleep(2000);
        telemetry.update();
        //telemetry.clear();
        waitForStart();
        time.reset();
        time_flip.reset();
        time_bascula.reset();

        time.startTime();
        time_flip.startTime();
        time_bascula.startTime();

        while (opModeIsActive()) {
            telemetry.addData("time sec ",time.seconds());
            telemetry.addData("time_flip sec ", time_flip.seconds());
            telemetry.addData("time_bascula sec ", time_bascula.seconds());
            telemetry.update();
            Drive();
            //robot.front_left_motor.getConnectionInfo()
//            rotor_b();
//            Elevator();
//            Flip();
//            bascula();
          //  Catch();
            idle();
        }
    }
}
