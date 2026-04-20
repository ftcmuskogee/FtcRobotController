package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Little Jimmy - Teleop", group = "PedroPascal")
public class DriveONLYTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LF");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LB");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RF");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RB");
        CRServo axonCR = hardwareMap.get(CRServo.class, "axonCR");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        axonCR.setDirection(CRServo.Direction.FORWARD);

        // If no driver input, the robot won't move
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double forward = gamepad1.right_trigger * 2;
            double reverse = gamepad1.left_trigger * 2;

            // Combine into one power value
            double power = forward - reverse;

            axonCR.setPower(power);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.45); // 0.45 for non-gripy wheels, 0.6 for gripy wheels
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("CRServo Power", power);
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("BL Power", backLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("BR Power", backRightPower);
            telemetry.update();


        }
    }
}