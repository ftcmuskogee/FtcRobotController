package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.aprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    aprilTagWebcam aprilTag = new aprilTagWebcam();


    // PID values (tune if needed)
    double kP = 0.03;
    double kD = 0.001;
    double lastError = 0;

    private double headingPID(double error) {
        double derivative = error - lastError;
        lastError = error;
        return (kP * error) + (kD * derivative);
    }

    @Override
    public void runOpMode() throws InterruptedException {


        // ---------------- MOTORS ----------------
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BR");

        DcMotor shooterMotor1 = hardwareMap.get(DcMotor.class, "S1");
        DcMotor shooterMotor2 = hardwareMap.get(DcMotor.class, "S2");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("NTK");

        Servo shootServo = hardwareMap.get(Servo.class, "SS");
        Servo hoodServo = hardwareMap.get(Servo.class, "HOOD");

        // Motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Zero power behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shootServo.scaleRange(0.15, 0.275);
        hoodServo.scaleRange(0.09, 0.21);
        aprilTag.init(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;

        shootServo.setPosition(0.9);
        hoodServo.setPosition(0.20);

        while (opModeIsActive()) {
            aprilTag.update();

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            boolean snapActive = gamepad1.right_bumper;

            if (snapActive) {

                AprilTagDetection tag20 = aprilTag.getTagByspecificId(20);
                AprilTagDetection tag24 = aprilTag.getTagByspecificId(24);

                AprilTagDetection targetTag = null;
                double desiredYaw = 0;

                if (tag20 != null) {
                    targetTag = tag20;
                    desiredYaw = 139;
                } else if (tag24 != null) {
                    targetTag = tag24;
                    desiredYaw = 35;
                }

                if (targetTag != null) {

                    double yawError = targetTag.ftcPose.yaw;
                    double correctedError = yawError - desiredYaw;

                    if (Math.abs(correctedError) > 1.5) {
                        rx = headingPID(correctedError);
                    } else {
                        rx = 0;
                    }

                    rx = Math.max(-0.4, Math.min(0.4, rx));

                    telemetry.addData("Snap Mode", "ACTIVE");
                    telemetry.addData("Tag ID", targetTag.id);
                    telemetry.addData("Yaw Error", yawError);
                    telemetry.addData("Corrected Error", correctedError);
                }
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double driveMult = 1;
            if (gamepad1.left_trigger >= 0.05) {
                driveMult = 0.35;
            } else if (gamepad1.right_trigger >= 0.05) {
                driveMult = 1.25;
            }

            frontLeftMotor.setPower(frontLeftPower * driveMult);
            backLeftMotor.setPower(backLeftPower * driveMult);
            frontRightMotor.setPower(frontRightPower * driveMult);
            backRightMotor.setPower(backRightPower * driveMult);


            if (gamepad2.right_trigger >= 0.025) {
                shooterMotor1.setPower(1);
                shooterMotor2.setPower(1);
            } else if (gamepad2.left_trigger >= 0.025) {
                shooterMotor1.setPower(-1);
                shooterMotor2.setPower(-1);
            } else {
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
            }

            if (gamepad2.a) {
                shootServo.setPosition(0);
            }
            if (gamepad2.b) {
                shootServo.setPosition(0.9);
            }


            if (gamepad2.dpad_down) {
                intakeMotor.setPower(1);
            } else if (gamepad2.dpad_up) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }


            if (gamepad2.x) {
                hoodServo.setPosition(1);
            }
            if (gamepad2.y) {
                hoodServo.setPosition(0);
            }

            telemetry.update();
        }
    }
}