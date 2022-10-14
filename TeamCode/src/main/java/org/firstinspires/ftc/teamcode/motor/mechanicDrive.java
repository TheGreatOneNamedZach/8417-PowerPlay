package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

public class mechanicDrive extends OpMode{

    DcMotor fR = null;
    DcMotor fL = null;
    DcMotor bR = null;
    DcMotor bL = null;
    double damper = 1; //This is to control the speed at which all the wheels turn


    @Override
    public void init() {
        DcMotor fR = hardwareMap.get(DcMotor.class, "Front Right");
        DcMotor fL = hardwareMap.get(DcMotor.class, "Front Left");
        DcMotor bR = hardwareMap.get(DcMotor.class, "Back Right");
        DcMotor bL = hardwareMap.get(DcMotor.class, "Back Left");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
    setPowerMechanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

    public void setPowerMechanum(double x, double y, double rot) {//rot is short for rotation
        //Code to calculate motor power
        double fRMotorPwr = x - y - rot;
        double fLMotorPwr = x + y + rot;
        double bRMotorPwr = x + y - rot;
        double bLMotorPwr = x - y + rot;

        double[] motorPwrs = {

                Math.abs(fRMotorPwr),
                Math.abs(fLMotorPwr),
                Math.abs(bRMotorPwr),
                Math.abs(bLMotorPwr)
        };

        Arrays.sort(motorPwrs);

        if (motorPwrs[3] != 0){
            fRMotorPwr = fRMotorPwr / motorPwrs[3];
            fLMotorPwr = fLMotorPwr / motorPwrs[3];
            bRMotorPwr = bRMotorPwr / motorPwrs[3];
            bLMotorPwr = bLMotorPwr / motorPwrs[3];
        }

        fR.setPower(fRMotorPwr / damper);
        fL.setPower(fLMotorPwr / damper);
        bR.setPower(bRMotorPwr / damper);
        bL.setPower(bLMotorPwr / damper);



    }
}