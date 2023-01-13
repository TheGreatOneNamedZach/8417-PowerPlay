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
        if(isLoop == 0){

        }
    }
}
