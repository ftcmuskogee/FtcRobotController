package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class EncoderShooter extends OpMode {

    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public DcMotor intake;

    public double farV = 2041.8773; // P = 13, F = 15 V NO LARGER THAN 2100!!!!!!!!!
    public double closeV = 1500; // P = 73, F = 14
    public double curTargetV = closeV;

    double P = 0;
    double F = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1; // 1st is 0, but start list at 1.0

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "S1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "S2");
        intake = hardwareMap.get(DcMotorEx.class, "NTK");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        telemetry.addLine("Initialized   :)");
    }

    @Override
    public void loop() {

        if (gamepad1.yWasPressed()) {
            if (curTargetV == farV) {
                curTargetV = closeV;
            } else {
                curTargetV = farV;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad2.dpad_up) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        if (gamepad2.right_stick_y >= 0.05) {
            curTargetV += -gamepad2.right_stick_y;
        } else if (gamepad2.right_stick_y <= -0.05) {
            curTargetV += -gamepad2.right_stick_y;
        }

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        shooter1.setVelocity(curTargetV);
        shooter2.setVelocity(curTargetV);

        double curV1 = shooter1.getVelocity();
        double curV2 = shooter2.getVelocity();

        double err1 = curTargetV - curV1;
        double err2 = curTargetV - curV2;

        telemetry.addData("Target Velocity", curTargetV);
        telemetry.addData("Current Velocity for Shooter 1","%.2f", curV1);
        telemetry.addData("Current Velocity for Shooter 2","%.2f", curV2);
        telemetry.addData("Error for Shooter 1", "%.2f", err1);
        telemetry.addData("Error for Shooter 2", "%.2f", err2);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad ^/v)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad </>)", F);
        telemetry.addData("Step Size","%.4f (B / Circle)", stepSizes[stepIndex]);
    }
}
