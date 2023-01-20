package org.firstinspires.ftc.teamcode.motor;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class autoSafetyNet extends OpMode {
    DcMotor fR = null;
    DcMotor fL = null;
    DcMotor bR = null;
    DcMotor bL = null;
    public ElapsedTime runtime = new ElapsedTime();
    int isLoop = 0;

    static final double TICKS_PER_REV = 1120;
    static final double wheelDiameter = 4.72441;
    static final double TICKS_PER_INCH = TICKS_PER_REV / (wheelDiameter * Math.PI);

    public void init(){
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);

        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        if(true) {
            //Should be code for left parking spot.
            if (isLoop == 0) {
                encoderDrive(.5, 24, 24);
            }
            if (runtime.time() >= 0.500 && isLoop == 1) {
                encoderDrive(.5, -5, 5);
            }
            if (runtime.time() >= 0.500 && isLoop == 2) {
                encoderDrive(.5, 24, 24);
            }
            if(runtime.time() >= .500 && isLoop == 3) {
                encoderDrive(.5, 5, -5);
            }
            if(runtime.time() >= .500 && isLoop == 4) {
                encoderDrive(.5, 12, 12);
            }
        }

        //Should be code for front parking spot

        if (false) {
            if(isLoop == 0) {
                encoderDrive(.5, 36, 36);
            }
        }

        //Should be code for right parking spot
        if(false) {
            if(isLoop == 0) {
                encoderDrive(.5, 24, 24);
            }
            if(runtime.time() >= .500 && isLoop == 1) {
                encoderDrive(.5, 5, -5);
            }
            if(runtime.time() >= .500 && isLoop == 2) {
                encoderDrive(.5, 24, 24);
            }
            if(runtime.time() >= .500 && isLoop == 3) {
                encoderDrive(.5, -5, 5);
            }
            if (runtime.time() >= .500 && isLoop == 4) {
                encoderDrive(.5, 12, 12);
            }
        }

        //If it just so happens to not see images or gets super confused;
        if(false) {
            if(isLoop == 0) {
                encoderDrive(.5, 5, -5);
            }
            if(runtime.time() >= .500 && isLoop == 1) {
                encoderDrive(.5, 24, 24);
            }
        }
        isLoop++;
    }




    public void encoderDrive(double speed, double leftInches, double rightInches) {
        if(!runOnce) {
            leftFrontTarget = 0;
            rightFrontTarget = 0;
            leftBackTarget = 0;
            rightBackTarget = 0;

            leftFrontTarget = /*fL.getCurrentPosition() +*/ (int) (leftInches * TICKS_PER_INCH);
            rightFrontTarget = /*fR.getCurrentPosition() +*/ (int) (rightInches * TICKS_PER_INCH);
            leftBackTarget = /*bL.getCurrentPosition() +*/ (int) (leftInches * TICKS_PER_INCH);
            rightBackTarget = /*bR.getCurrentPosition() +*/ (int) (rightInches * TICKS_PER_INCH);

            // Telling the motors how many ticks I want them to go and then to stop
            if(leftBackTarget < 0) {
                bL.setTargetPosition(leftBackTarget - (int) TICKS_PER_REV);
                fL.setTargetPosition(leftFrontTarget - (int) TICKS_PER_REV);
            } else {
                bL.setTargetPosition(leftBackTarget + (int) TICKS_PER_REV);
                fL.setTargetPosition(leftFrontTarget + (int) TICKS_PER_REV);
            }

            if(rightBackTarget < 0) {
                fR.setTargetPosition(rightFrontTarget - (int) TICKS_PER_REV);
                bR.setTargetPosition(rightBackTarget - (int) TICKS_PER_REV);
            } else {
                fR.setTargetPosition(rightFrontTarget + (int) TICKS_PER_REV);
                bR.setTargetPosition(rightBackTarget + (int) TICKS_PER_REV);
            }

            // Changes motor mode to "run to position"
            //fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Sending power to motors
            if(rightInches < 0) {
                fR.setPower(-speed);
            } else {
                fR.setPower(speed);
            }
            bR.setPower(speed);
            bL.setPower(speed);
            fL.setPower(speed);
            runOnce = true;
        }

        if(((Math.round(Math.abs(fL.getCurrentPosition() + bL.getCurrentPosition()))/2) >= Math.abs(leftFrontTarget))/*((Math.round(Math.abs(fL.getCurrentPosition() + bL.getCurrentPosition()))/2) >= Math.abs(leftFrontTarget)) && Math.round(Math.abs(bR.getCurrentPosition())) >= Math.abs(rightBackTarget)*/) {
            if (Math.round(Math.abs(bR.getCurrentPosition())) >= Math.abs(rightBackTarget)) {
                // Takes the average, absolute value of both sides and makes sure one of them is above their target position
                // Stops the motors after they reach the position
                fR.setPower(0);
                bR.setPower(0);
                fL.setPower(0);
                bL.setPower(0);

                ififrun = true;
                // Changed the motor type to "run using encoders"
                //fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // Resets encoder ticks
                fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                runtime.reset();
                runOnce = false;
                ifLoopCount++;
            }
        }
    }
}
}
