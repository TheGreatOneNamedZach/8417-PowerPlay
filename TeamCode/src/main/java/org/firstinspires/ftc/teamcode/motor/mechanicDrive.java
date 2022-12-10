package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public ElapsedTime timer = new ElapsedTime();

    DcMotor elevator = null;
    Servo claw = null;
    DigitalChannel digitalTouch;
    double motorPower; // Declares a double for telemetry and motor use
    int motor1Pos = 0; // This saves the motor position when it is first at rest
    boolean firstLoop = false;
    boolean isPressed = false;

    double p;
    double i = 0; // Must never be null
    double d;
    double k_p = 0.2; // Rate of power change based on error
    double k_i = 0;
    double k_d = 3;
    double lastTime = -1;
    double lastError = -1;

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
                    slideControl(motor1Pos);
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
        } else if(gamepad2.dpad_left){
            elevator.setPower(0.6);
            elevator.setTargetPosition(2260);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if(gamepad2.dpad_up){
            elevator.setPower(0.6);
            elevator.setTargetPosition(3600);
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
            telemetry.addData("Claw", "Closed");
        }else{
            claw.setPosition(.42);
            telemetry.addData("Claw", "Open");
        }
    }

    /*
    * If added, this code will presumably allow the right bumper to be used as a toggle (on/off)
    * switch, reducing the chances of user error when piloting the robot.
    *
    * This portion specifically is the on/off portion. It uses the controller's built-in boolean
    * value together with whether or not the claw itself is open or closed to change the value to
    * opposite of what it is.

    boolean closed = false;

    public void clawControl(boolean a) {
        if(a && !closed){
            closed = true;
        } else if(a && closed){
            closed = false;
        }

    * This portion of the code (still under the claw control function) opens and closes the claw
    * itself

        if(!closed){
            claw.setPosition(.42);
            telemetry.addData(caption:"Claw", value:"Open");
        } else if(closed){
            claw.setPosition(.64);
            telemetry.addData(caption:"Claw", value:"Open");
        }
     }
    * The user can now use a single press on the bumper with this code, because the robot now
    * understands whether or not the claw is open or closed, by knowing that the opposite is, in
    * fact, quite true. Knowing whether it is not opened or not closed, allows the robot, at a push
    * of a button, to say that now it is that which it wasn't. Knowing that is wasn't what it now is,
    * the robot can now say that it is what it isn't, allowing it to move and hold a servo motor
    * in the position that the robot now knows it needs to be in.
    */

    public void slideControl(int targetPos) {
        /*
         * The linear slide knows where it is at all times.
         * It knows this because it knows where it isn't.
         * By subtracting where it is from where it isn't,
         * or where it isn't from where it is (whichever is greater),
         * it obtains a difference, or deviation.
         * The PID system uses deviations to generate corrective commands
         * to drive the linear side from a position where it is to a position where it isn't,
         * and arriving at a position where it wasn't, it now is.
         * Consequently, the position where it is, is now the position that it wasn't,
         * and it follows that the position that it was, is now the position that it isn't.
         * In the event that the position that it is in is not the position that it wasn't,
         * the PID system has acquired a variation, the variation being the difference
         * between where the linear slide is, and where it wasn't.
         * If variation is considered to be a significant factor,
         * it too may be corrected by the PID.
         * However, the linear slide must also know where it was.
         * The linear slide PID system works as follows.
         * Because a variation has modified some of the information the linear slide has obtained,
         * it is not sure just where it is.
         * However, it is sure where it isn't, within reason, and it knows where it was.
         * It now subtracts where it should be from where it wasn't,
         * or vice-versa, and by differentiating this from the algebraic sum of where it shouldn't be,
         * and where it was, it is able to obtain the deviation and its variation, which is called error.
         */
        double currentTime = timer.milliseconds();
        double currentError = targetPos - elevator.getCurrentPosition();

        p = k_p * currentError;
        i += k_i * (currentError * (currentTime - lastTime));
        i = Math.max(i, -1);
        i = Math.min(i, 1);
        d = k_d * (currentError - lastError) / (currentTime - lastTime);

        elevator.setPower(0.01 * (p + i + d));
        telemetry.addData("",0.01 * (p + i + d));

        lastTime = currentTime;
        lastError = currentError;
    }
}