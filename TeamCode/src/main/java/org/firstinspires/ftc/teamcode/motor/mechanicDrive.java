package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

@TeleOp(name = "Mechanum Drive", group = "e")
public class mechanicDrive extends OpMode{

    DcMotor fR = null;
    DcMotor fL = null;
    DcMotor bR = null;
    DcMotor bL = null;
    double damper = 0.75; //This is to control the percent of energy being applied to the motors.


    @Override
    public void init() {
        fR = hardwareMap.get(DcMotor.class, "Front Right");
        fL = hardwareMap.get(DcMotor.class, "Front Left");
        bR = hardwareMap.get(DcMotor.class, "Back Right");
        bL = hardwareMap.get(DcMotor.class, "Back Left");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
    setPowerMechanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

    public void setPowerMechanum(double x, double y, double rot) {//rot is short for rotation
        x = x * 1.1;
        //Code to calculate motor power
        double ratio = Math.max((Math.abs(y) + Math.abs(x) + Math.abs(rot)), 1);
        double fRMotorPwr = (y - x - rot) / ratio;
        double fLMotorPwr = (y + x + rot) / ratio;
        double bRMotorPwr = (y + x - rot) / ratio;
        double bLMotorPwr = (y - x + rot) / ratio;
        /* This is the first version of algorithms
        double fRMotorPwr = x - y - rot;
        double fLMotorPwr = x + y + rot;
        double bRMotorPwr = x + y - rot;
        double bLMotorPwr = x - y + rot;
         */

        /*
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
        }*/
        telemetry.addData("fRMotorPwr", fRMotorPwr);
        telemetry.addData("fLMotorPwr", fLMotorPwr);
        telemetry.addData("bRMotorPwr", bRMotorPwr);
        telemetry.addData("bLMotorPwr", bLMotorPwr);
        try {
            fR.setPower(fRMotorPwr * damper);
            fL.setPower(fLMotorPwr * damper);
            bR.setPower(bRMotorPwr * damper);
            bL.setPower(bLMotorPwr * damper);
        } catch(Exception e) {
            telemetry.addData("Error", e);
        }



    }
}