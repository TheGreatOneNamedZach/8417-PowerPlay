package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.text.DecimalFormat;

@TeleOp(name = "Mechanum Drive", group = "e")
public class mechanicDrive extends OpMode{

    DcMotor fR = null;
    DcMotor fL = null;
    DcMotor bR = null;
    DcMotor bL = null;
    double totalSpeed = 0.75; //This is to control the percent of energy being applied to the motors.
    double slowSpeed = 0.50; // x% of whatever speed totalSpeed is
    static final DecimalFormat df = new DecimalFormat("0.00"); // for rounding

    @Override
    public void init() {
        fR = hardwareMap.get(DcMotor.class, "Front Right");
        fL = hardwareMap.get(DcMotor.class, "Front Left");
        bR = hardwareMap.get(DcMotor.class, "Back Right");
        bL = hardwareMap.get(DcMotor.class, "Back Left");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("", "█");
        telemetry.addData("", "↑");
    }

    @Override
    public void loop() {
        setPowerMechanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        if(gamepad1.right_bumper) {
            slowSpeed = 1.00;
        } else {
            slowSpeed = 0.50;
        }
    }

    public void setPowerMechanum(double x, double y, double rot) {//rot is short for rotation
        x = x * 1.1;
        y = -y;
        rot = -rot;
        //Code to calculate motor power
        double ratio = Math.max((Math.abs(y) + Math.abs(x) + Math.abs(rot)), 1);
        double fRMotorPwr = (y - x + rot) / ratio;
        double fLMotorPwr = (-y - x + rot) / ratio;
        double bRMotorPwr = (-y - x - rot) / ratio;
        double bLMotorPwr = (y - x - rot) / ratio;

        telemetry.addData("fRMotorPwr", df.format(fRMotorPwr));
        telemetry.addData("fLMotorPwr", df.format(fLMotorPwr));
        telemetry.addData("bRMotorPwr", df.format(bRMotorPwr));
        telemetry.addData("bLMotorPwr", df.format(bLMotorPwr));

        fR.setPower(fRMotorPwr * totalSpeed * slowSpeed);
        fL.setPower(fLMotorPwr * totalSpeed * slowSpeed);
        bR.setPower(bRMotorPwr * totalSpeed * slowSpeed);
        bL.setPower(bLMotorPwr * totalSpeed * slowSpeed);
    }
}