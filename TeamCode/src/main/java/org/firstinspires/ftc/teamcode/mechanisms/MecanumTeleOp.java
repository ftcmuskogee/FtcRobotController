package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.java_websocket.framing.ContinuousFrame;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            // Declare our motors
            // Make sure your ID's match your configuration
            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FL");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("BL");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("FR");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("BR");

            DcMotor shooterMotor1 = hardwareMap.get(DcMotor.class, "S1");
            DcMotor shooterMotor2 = hardwareMap.get(DcMotor.class, "S2");
            DcMotor intakeMotor = hardwareMap.dcMotor.get("NTK");

            Servo shootServo = hardwareMap.get(Servo.class, "SS");
            Servo hoodServo = hardwareMap.get(Servo.class, "HOOD");

            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.
            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            shootServo.setDirection(Servo.Direction.FORWARD);
            hoodServo.setDirection(Servo.Direction.FORWARD);

            // If no driver input, the robot won't move
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set limits to the servos.
            shootServo.scaleRange(0.15, 0.275); // ?
            hoodServo.scaleRange(0.09, 0.21);

            waitForStart();

            if (isStopRequested()) return;

            // Ensures that the servos correct themselves AFTER the robot can move without fouls (at start of TeleOp)
            shootServo.setPosition(0.9); // ?
            hoodServo.setPosition(0.20);

            while (opModeIsActive()) {

                // Drive system
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                // Shooting system
                if (gamepad2.right_trigger >= 0.025) {;
                    shooterMotor1.setPower(1);
                    shooterMotor2.setPower(1);
                } else if (gamepad2.left_trigger >= 0.025) {
                    shooterMotor1.setPower(-1);
                    shooterMotor2.setPower(-1);
                } else {
                    shooterMotor1.setPower(0);
                    shooterMotor2.setPower(0);
                }

                /*if (gamepad2.aWasPressed()){
                    shootServo.setPosition(0);
                    intakeMotor.setPower(0.9);
                    sleep(1000);
                    intakeMotor.setPower(0);
                    shootServo.setPosition(0.9);
                }
                if (gamepad2.bWasPressed()){
                    shootServo.setPosition(0);
                    intakeMotor.setPower(0.9);
                    sleep(300);
                    intakeMotor.setPower(0);
                    shootServo.setPosition(0.9);
                }*/

                if (gamepad2.aWasPressed()){
                    shootServo.setPosition(0);
                }
                if (gamepad2.bWasPressed()){
                    shootServo.setPosition(0.9);
                }

                // Intake system
                if (gamepad2.dpad_down) {
                    intakeMotor.setPower(1); // IN
                } else if (gamepad2.dpad_up) {
                    intakeMotor.setPower(-1); // OUT
                } else {
                    intakeMotor.setPower(0);
                }

                // Hood system
                if (gamepad2.xWasPressed()) {
                    hoodServo.setPosition(1);
                } else if (gamepad2.yWasPressed()) {
                    hoodServo.setPosition(0);
                }

                // Controlled drive system
                double driveMult = 1;
                boolean driveSlowed = false;
                boolean driveStronger = false;
                if (gamepad1.left_trigger >= 0.05) {
                    driveMult = 0.35;
                    driveSlowed = true;
                } else if (gamepad1.right_trigger >= 0.05) {
                    driveMult = 1.25;
                    driveStronger = true;
                } else {
                    driveMult = 1;
                    driveSlowed = false;
                    driveStronger = false;
                }

                frontLeftMotor.setPower(frontLeftPower * driveMult);
                backLeftMotor.setPower(backLeftPower * driveMult);
                frontRightMotor.setPower(frontRightPower * driveMult);
                backRightMotor.setPower(backRightPower * driveMult);

                telemetry.addData("Precision Driving", driveSlowed);
                telemetry.addData("Stronger Driving", driveStronger);
                telemetry.addData("FL Power", frontLeftPower);
                telemetry.addData("BL Power", backLeftPower);
                telemetry.addData("FR Power", frontRightPower);
                telemetry.addData("BR Power", backRightPower);
                telemetry.update();

            }
        }
    }