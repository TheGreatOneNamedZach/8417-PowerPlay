package org.firstinspires.ftc.teamcode.motor;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.tF8417Main;

import java.util.Objects;

@Autonomous
public class autoSafetyNet extends OpMode {
    DcMotor fR = null;
    DcMotor fL = null;
    DcMotor bR = null;
    DcMotor bL = null;
    public ElapsedTime runtime = new ElapsedTime();
    int isLoop = 0;
    double length = 1.0;
    double slow = .5;
    org.firstinspires.ftc.teamcode.vision.tF8417Main detector = new tF8417Main();
    String duck = null; //Zach misspoke, so here it is now.
    String tempDuck = null;

    public void init(){
        fR = hardwareMap.get(DcMotor.class, "Front Right");
        fL = hardwareMap.get(DcMotor.class, "Front Left");
        bR = hardwareMap.get(DcMotor.class, "Back Right");
        bL = hardwareMap.get(DcMotor.class, "Back Left");

        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        detector.init(this);
        detector.initVuforia();
        detector.initTfod();
    }

    public void init_loop(){
        tempDuck = detector.imageReturn();
        if(tempDuck != null){
            duck = tempDuck;
        }
        telemetry.addData("label", duck);
    }

    @Override
    public void loop() {
        telemetry.addData("Time elapsed", runtime.time());
        telemetry.addData("isLoop", isLoop);





        if(Objects.equals(duck, "Turtle")) {
            //Should be code for left parking spot.
            if(isLoop == 0){
                encoderDrive(length, 0, 0, 0);
            }
            if(isLoop == 1){
                encoderDrive(.2, 1, 0, 0);
            }
            if(isLoop == 2){
                encoderDrive(.2, 0,0,0);
            }
            if (isLoop == 3) {
                encoderDrive(length, 0,  1, 0);
            }
            if(isLoop == 4){
                encoderDrive(.2, 0,0,0);
            }
            if (isLoop == 5) {
                encoderDrive(length,1, 0, 0);
            }
            if(isLoop == 6){
                encoderDrive(length, 0,0,0);
            }
        }


        //Should be code for front parking spot

        if (Objects.equals(duck, "Robot")) {
            if(isLoop == 0){
                encoderDrive(length, 0,0,0);
            }
            if(isLoop == 1) {
                encoderDrive(length + .5, 1, 0, 0);
            }
            if(isLoop == 2){
                encoderDrive(length,0,0,0);
            }
        }

        //Should be code for right parking spot
        if(Objects.equals(duck, "Handsaw")) {
            if(isLoop == 0){
                encoderDrive(length,0,0,0);
            }
            if(isLoop == 1) {
                encoderDrive(.2, 1, 0, 0);
            }
            if(isLoop == 2){
                encoderDrive(.2,0,0,0);
            }
            if(isLoop == 3) {
                encoderDrive(length, 0, -1, 0);
            }
            if(isLoop == 4){
                encoderDrive(.2,0,0,0);
            }
            if(isLoop == 5) {
                encoderDrive(length, 1, 0, 0);
            }
            if(isLoop == 6) {
                encoderDrive(.2, 0, 0, 0);
            }
        }
        if (Objects.equals(duck, null)) {
            if(isLoop == 0){
                encoderDrive(length, 0,0,0);
            }
            if(isLoop == 1) {
                encoderDrive(length + .5, 1, 0, 0);
            }
            if(isLoop == 2){
                encoderDrive(length,0,0,0);
            }
        }

    }



        public void encoderDrive(double lengthy, double x, double y, double rot) {//rot is short for rotation
            x = x * 1.1;
            y = -y;
            rot = -rot;
            //Code to calculate motor power
            double ratio = Math.max((Math.abs(y) + Math.abs(x) + Math.abs(rot)), 1);
            double fRMotorPwr = (y - x + rot) / ratio;
            double fLMotorPwr = (-y - x + rot) / ratio;
            double bRMotorPwr = (-y - x - rot) / ratio;
            double bLMotorPwr = (y - x - rot) / ratio;

            telemetry.addData("fRMotorPwr", fRMotorPwr);
            telemetry.addData("fLMotorPwr", fLMotorPwr);
            telemetry.addData("bRMotorPwr", bRMotorPwr);
            telemetry.addData("bLMotorPwr", bLMotorPwr);

            fR.setPower(fRMotorPwr * slow);
            fL.setPower(fLMotorPwr * slow);
            bR.setPower(bRMotorPwr * slow);
            bL.setPower(bLMotorPwr * slow);
            if(runtime.time() >= lengthy){
                isLoop++;
                runtime.reset();
            }
        }
}