package org.firstinspires.ftc.teamcode.Autonomo;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class testeRR extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Pose2d startPose = new Pose2d(12, -62, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(new Vector2d(-53,-53), Math.PI / 4)
                        .strafeTo(new Vector2d(-57,-57))
                        .strafeToLinearHeading(new Vector2d(-36, -34), Math.PI)
                        .strafeTo(new Vector2d(-36, -25))
                        .strafeToLinearHeading(new Vector2d(-53,-53), Math.PI / 4)
                        .strafeTo(new Vector2d(-57,-57))
                        .strafeToLinearHeading(new Vector2d(-45, -24), Math.PI).strafeToLinearHeading(new Vector2d(-53,-53), Math.PI / 4)
                        .strafeTo(new Vector2d(-57,-57))
                        .strafeToLinearHeading(new Vector2d(53,-60), Math.PI)
                        .build()
        );
    }
}
