package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.DecimalFormat;

@TeleOp(name = "Mechanum Drive", group = "e")
public class mechanicDrive extends OpMode{

    DcMotor fR = null;
    DcMotor fL = null;
    DcMotor bR = null;
    DcMotor bL = null;
    static double totalSpeed = 0.75; //This is to control the percent of energy being applied to the motors.
    double slowSpeed = 0.50; // x% of whatever speed totalSpeed is
    static final DecimalFormat df = new DecimalFormat("0.00"); // for rounding

    DcMotor elevator = null;
    Servo claw = null;
    DigitalChannel digitalTouch;
    double motorPower; // Declares a double for telemetry and motor use
    int motor1Pos = 0; // This saves the motor position when it is first at rest
    boolean firstLoop = false;
    boolean isPressed = false;

    @Override
    public void init() {
        fR = hardwareMap.get(DcMotor.class, "Front Right");
        fL = hardwareMap.get(DcMotor.class, "Front Left");
        bR = hardwareMap.get(DcMotor.class, "Back Right");
        bL = hardwareMap.get(DcMotor.class, "Back Left");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        /*
        telemetry.addData("", "█");
        telemetry.addData("", "↑");
         */

        elevator = hardwareMap.get(DcMotor.class, "Elevator");
        claw = hardwareMap.get(Servo.class, "Claw");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        setPowerMechanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        if(gamepad1.right_bumper) {
            slowSpeed = 1.00;
        } else {
            slowSpeed = 0.50;
        }

        telemetry.addData("GP2LS", gamepad2.left_stick_y);
        motorPower = (gamepad2.left_stick_y * -0.75);
        if (!(isPressed && gamepad2.left_stick_y > 0)) {
            elevator.setPower(motorPower);
            telemetry.addData("Elevator", motorPower); // motor1 power
            telemetry.addData("Pos", elevator.getCurrentPosition());

            if (gamepad2.left_stick_y == 0 && !isPressed) { // If the joystick is at rest AND the limit switch is not pressed...
                if (!firstLoop) { // ...and the motor was NOT at rest in the last loop...
                    motor1Pos = elevator.getCurrentPosition(); // ...get the motor position.
                    firstLoop = true;
                /*
                This says the motor has been at rest before.
                This is so the motor position is not constantly reset every loop iteration
                while the motor is at rest.
                 */
                } else if (elevator.getCurrentPosition() < motor1Pos) { // ...and the motor WAS at rest in the last loop...
                    elevator.setTargetPosition(motor1Pos); // Sets the target position
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // Makes the positive direction the motor runs
                    // towards the same direction you want it to go
                    elevator.setPower(0.1);
                }
            }
            else { // If the joystick is NOT at rest...
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                firstLoop = false;
                elevator.setPower(motorPower);
            }
        }
        if(gamepad2.dpad_down){
            elevator.setPower(0.6);
            elevator.setTargetPosition(1400);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        clawControl(gamepad2.right_bumper);

        if (digitalTouch.getState()) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
            isPressed = false;
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
            isPressed = true;
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

    public void clawControl(boolean a) { //"a" is the variable being passed in for the claw to close/open
        if(a){
            claw.setPosition(.64);
            telemetry.addData("Claw", "Open");
        }else{
            claw.setPosition(.42);
            telemetry.addData("Claw", "Closed");
        }
    }
}