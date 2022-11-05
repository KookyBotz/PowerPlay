package org.firstinspires.ftc.teamcode.opmode.test.System;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
@Disabled

@TeleOp
public class Test extends OpMode {
    Robot robot;
    @Override
    public void init(){
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop(){
        if(gamepad1.a){
            robot.drivetrain.leftFrontModule.setMotorPower(1);
            robot.drivetrain.leftFrontModule.setTargetRotation(0);
        }else{
            robot.drivetrain.leftFrontModule.setMotorPower(0);
            robot.drivetrain.leftFrontModule.setTargetRotation(1);
        }
        if(gamepad1.b){
            robot.drivetrain.leftRearModule.setMotorPower(1);
            robot.drivetrain.leftRearModule.setTargetRotation(0);
        }else{
            robot.drivetrain.leftRearModule.setMotorPower(0);
            robot.drivetrain.leftRearModule.setTargetRotation(1);
        }
        if(gamepad1.x){
            robot.drivetrain.rightFrontModule.setMotorPower(1);
            robot.drivetrain.rightFrontModule.setTargetRotation(0);
        }else{
            robot.drivetrain.rightFrontModule.setMotorPower(0);
            robot.drivetrain.rightFrontModule.setTargetRotation(1);
        }
        if(gamepad1.y){
            robot.drivetrain.rightRearModule.setMotorPower(1);
            robot.drivetrain.rightRearModule.setTargetRotation(0);
        }else{
            robot.drivetrain.rightRearModule.setMotorPower(0);
            robot.drivetrain.rightRearModule.setTargetRotation(1);
        }
    }
}
