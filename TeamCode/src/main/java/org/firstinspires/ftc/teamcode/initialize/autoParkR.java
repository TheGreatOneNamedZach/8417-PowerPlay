package org.firstinspires.ftc.teamcode.initialize;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.other.redr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.other.redr.trajectorysequence.TrajectorySequence;

public class autoParkR extends LinearOpMode {
    TrajectorySequence LeftParking;
    Boolean autoFinished = false;
    public void runOpMode(){
        SampleMecanumDrive drive;
        drive = new SampleMecanumDrive(hardwareMap);
        while (opModeInInit()) {
            LeftParking = drive.trajectorySequenceBuilder(new Pose2d(36.00, -65.00, Math.toRadians(90.00)))
                    .splineTo(new Vector2d(13.00, -54.00), Math.toRadians(90.00))
                    .splineTo(new Vector2d(13.00, -37.00), Math.toRadians(90.00))
                    .build();
            drive.setPoseEstimate(LeftParking.start());
        }
        waitForStart();

        while (opModeIsActive() && !autoFinished) {
            drive.followTrajectorySequence(LeftParking);
            autoFinished = true;
        }
    }
}