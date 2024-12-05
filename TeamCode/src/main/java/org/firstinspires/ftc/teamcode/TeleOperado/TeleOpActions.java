package org.firstinspires.ftc.teamcode.TeleOperado;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;


import java.util.ArrayList;
import java.util.List;

public class TeleOpActions
{
    private List<Action> runningActions = new ArrayList<>();

    public void addToQueue(Action action)
    {
        runningActions.add(action);
    }

    public void runActions(TelemetryPacket packet)
    {
        List<Action> newActions = new ArrayList<>();

        for (Action action : runningActions)
        {
            action.preview(packet.fieldOverlay());
            if (action.run(packet))
            {
                newActions.add(action);
            }
        }

        runningActions = newActions;

    }

}
