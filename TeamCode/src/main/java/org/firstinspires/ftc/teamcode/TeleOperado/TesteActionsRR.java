package org.firstinspires.ftc.teamcode.TeleOperado;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

        TeleOpActions teleOpActions = new TeleOpActions();

        FtcDashboard ftcDashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive())
        {
            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad1.a)
            {
                teleOpActions.addToQueue(new SequentialAction(outtake.servoToOut()));
            }

            if (gamepad1.b)
            {
                teleOpActions.addToQueue(new SequentialAction(outtake.servoToIn()));
            }

            teleOpActions.runActions(packet);

            ftcDashboard.sendTelemetryPacket(packet);
        }
    }
}
