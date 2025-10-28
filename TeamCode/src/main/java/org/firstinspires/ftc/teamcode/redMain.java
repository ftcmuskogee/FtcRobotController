package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
public class redMain extends LinearOpMode {
    @Override
        public void runOpMode() throws InterruptedException {
            // Declare our motors
            // Make sure your ID's match your configuration
            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LF");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("LB");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("RF");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("RB");
            DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "SM");
            Servo servo = hardwareMap.get(Servo.class, "servo");

            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.
            frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            servo.setPosition(0.05);


            waitForStart();

            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(-1);
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
            sleep(100);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            shooterMotor.setPower(1);
            sleep(100);
            servo.setPosition(0.0);
            sleep(500);
            servo.setPosition(0.05);
            servo.setPosition(0.0);
            sleep(500);
            servo.setPosition(0.05);
            servo.setPosition(0.0);
            sleep(500);
            servo.setPosition(0.05);
            sleep(1);
            shooterMotor.setPower(0);
            sleep(1);
            frontRightMotor.setPower(1);
            backRightMotor.setPower(-1);
            frontLeftMotor.setPower(-1);
            backLeftMotor.setPower(1);
            sleep(1000);
        }



    }
