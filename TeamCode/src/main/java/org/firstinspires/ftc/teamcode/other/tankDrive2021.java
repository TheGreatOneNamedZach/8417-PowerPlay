package org.firstinspires.ftc.teamcode.other;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="2021 Tank Drive", group="Other")
public class tankDrive2021 extends OpMode{
    private DcMotor motor1 = null; // Declares a motor for general use
    double motorPower; // Declares a double for telemetry and motor use
    int motor1Pos = 0; // This saves the motor position when it is first at rest
    boolean firstLoop = false;

    @Override
    // Assigns the variable "motor1" to the port in the phone config
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop() {
        telemetry.addData("Runtime", Math.round(getRuntime()) + " sec");
        motorPower = gamepad1.left_stick_x;
        motor1.setPower(motorPower); // Gamepad 1 left joystick X axis controls motor1
        telemetry.addData("Motor1", motorPower); // motor1 power

        if(gamepad1.left_stick_x == 0) { // If the joystick is at rest...
            if(!firstLoop) { // ...and the motor was NOT at rest in the last loop...
                motor1Pos = motor1.getCurrentPosition(); // ...get the motor position.
                firstLoop = true;
                /*
                This says the motor has been at rest before.
                This is so the motor postion is not constantly reset every loop iteration
                while the motor is at rest.
                 */
            } else { // ...and the motor WAS at rest in the last loop...
                if((motor1.getCurrentPosition() < motor1Pos - 5) || (motor1.getCurrentPosition() > motor1Pos + 5)) {
                    // ...and the motor is no longer at the target position...
                    motor1.setTargetPosition(motor1Pos); // Sets the target position
                    motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // Makes the positive direction the motor runs
                    // towards the same direction you want it to go
                    motor1.setPower(1);
                }
            }

            if((motor1.getCurrentPosition() >= motor1Pos - 5) && (motor1.getCurrentPosition() <= motor1Pos + 5)) {
                // ...and the motor is at the target position...
                // ...stop the robot.
                motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor1.setPower(0);
            }

        } else { // If the joystick is NOT at rest...
            firstLoop = false;
        }


        /*
        if (MotorPlace == (MotorPlace - 5)) {
            if((motorPower) == (null)) {
                motor1 = motor1++;
            }
        }*/
    }
}