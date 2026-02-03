package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
//@Disabled
public class FlywheelTunePF extends OpMode {

    public DcMotorEx Shoot1;
    public DcMotorEx Shoot2;

    public double highV = 1500;
    public double lowV = 900;

    double curV = highV;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001}; // counts 0,1,2,3,4
    int stepIndex = 1;


    @Override
    public void init() {
        Shoot1 = hardwareMap.get(DcMotorEx.class, "S1");
        Shoot2 = hardwareMap.get(DcMotorEx.class, "S2");

        Shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shoot1.setDirection(DcMotorSimple.Direction.FORWARD);
        Shoot2.setDirection(DcMotorSimple.Direction.REVERSE);


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        Shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Initialized");

    }
    @Override
    public void loop() {

        if (gamepad1.yWasPressed()) {
            if (curV == highV) {
                curV = lowV;
            } else {
                curV = highV;
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

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        Shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        Shoot1.setVelocity(curV);
        Shoot2.setVelocity(curV);

        double nowV1 = Shoot1.getVelocity();
        double nowV2 = Shoot2.getVelocity();

        double error1 = curV - nowV1;
        double error2 = curV - nowV2;


        telemetry.addData("Target Velocity", curV);
        telemetry.addData("Current S1 Velocity", nowV1);
        telemetry.addData("Current V2 Velocity", nowV2);
        telemetry.addData("Error of S1", error1);
        telemetry.addData("Error of S2", error2);
        telemetry.addLine("-------------------------------------------------");
        telemetry.addData("Current P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Current F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
