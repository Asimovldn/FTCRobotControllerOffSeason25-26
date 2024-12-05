package org.firstinspires.ftc.teamcode.TeleOperado;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;

@TeleOp
public class TesteActionsRR extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Outtake outtake = new Outtake(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(outtake.servoToOut()));

        while (!isStopRequested())
        {

        }
    }
}
