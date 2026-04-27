package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import android.util.Size;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    // PID
    double kP = 0.03;
    double kD = 0.001;
    double lastError = 0;

    private double headingPID(double error) {
        double derivative = error - lastError;
        lastError = error;
        return (kP * error) + (kD * derivative);
    }

    @Override
    public void runOpMode() {

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

        // Directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shootServo.scaleRange(0.15, 0.275);
        hoodServo.scaleRange(0.09, 0.21);

        shootServo.setPosition(0.9);
        hoodServo.setPosition(0.20);

        // ---------------- APRILTAG (LOW POWER) ----------------
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .build();

        // lowest power
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480)) // lowest power
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .addProcessor(aprilTag)
                .build();

        // Start OFF
        visionPortal.setProcessorEnabled(aprilTag, false);

        boolean visionEnabled = false;
        int frameSkip = 0;

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- MAIN LOOP ----------------
        while (opModeIsActive()) {

            // -------- Vision Toggle --------
            AprilTagDetection targetTag = null;

            if (gamepad1.right_bumper) {
                visionPortal.resumeStreaming();
                visionPortal.setProcessorEnabled(aprilTag, true);
                visionEnabled = true;
            } else if (!gamepad1.right_bumper && targetTag == null) {
                visionPortal.setProcessorEnabled(aprilTag, false);
                visionPortal.stopStreaming();
                visionEnabled = false;
            }

            // -------- AprilTag Detection --------

            if (visionEnabled) {
                frameSkip++;

                if (frameSkip % 3 == 0) {
                    for (AprilTagDetection tag : aprilTag.getDetections()) {
                        if (tag.id == 20 || tag.id == 24) {
                            targetTag = tag;
                            break;
                        }
                    }
                }
            }

            // -------- Driving --------
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            // Snap-to-target
            if (gamepad1.right_bumper && targetTag != null) {

                double desiredYaw = (targetTag.id == 20) ? 139 : 39;

                double yawError = targetTag.ftcPose.yaw;
                double correctedError = yawError - desiredYaw;

                if (Math.abs(correctedError) > 1.5) {
                    rx = headingPID(correctedError);
                } else {
                    rx = 0;
                }

                rx = Math.max(-0.4, Math.min(0.4, rx));

                telemetry.addData("Snap", "ACTIVE");
                telemetry.addData("Tag", targetTag.id);
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double fl = (y + x + rx) / denominator;
            double bl = (y - x + rx) / denominator;
            double fr = (y - x - rx) / denominator;
            double br = (y + x - rx) / denominator;

            double driveMult = 1;
            if (gamepad1.left_trigger > 0.05) driveMult = 0.35;
            if (gamepad1.right_trigger > 0.05) driveMult = 1.25;

            frontLeftMotor.setPower(fl * driveMult);
            backLeftMotor.setPower(bl * driveMult);
            frontRightMotor.setPower(fr * driveMult);
            backRightMotor.setPower(br * driveMult);

            // -------- Shooter --------
            if (gamepad2.right_trigger > 0.05) {
                shooterMotor1.setPower(1);
                shooterMotor2.setPower(1);
            } else if (gamepad2.left_trigger > 0.05) {
                shooterMotor1.setPower(-1);
                shooterMotor2.setPower(-1);
            } else {
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
            }

            // Shoot servo
            if (gamepad2.a) shootServo.setPosition(0);
            if (gamepad2.b) shootServo.setPosition(0.9);

            // Intake
            if (gamepad2.dpad_down) intakeMotor.setPower(1);
            else if (gamepad2.dpad_up) intakeMotor.setPower(-1);
            else intakeMotor.setPower(0);

            // Hood
            if (gamepad2.x) hoodServo.setPosition(1);
            if (gamepad2.y) hoodServo.setPosition(0);

            telemetry.update();
        }
    }
}