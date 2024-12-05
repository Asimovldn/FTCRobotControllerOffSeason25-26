package com.example.meepmeepvisualizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepVisualizer {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -62, 0))
                .strafeToLinearHeading(new Vector2d(-53,-53), Math.PI / 4)
                .strafeTo(new Vector2d(-57,-57))
                .strafeToLinearHeading(new Vector2d(-36, -34), Math.PI)
                .strafeTo(new Vector2d(-36, -25))
                .strafeToLinearHeading(new Vector2d(-53,-53), Math.PI / 4)
                .strafeTo(new Vector2d(-57,-57))
                .strafeToLinearHeading(new Vector2d(-45, -24), Math.PI).strafeToLinearHeading(new Vector2d(-53,-53), Math.PI / 4)
                .strafeTo(new Vector2d(-57,-57))
                .strafeToLinearHeading(new Vector2d(51,-48), Math.PI)
                .strafeTo(new Vector2d(50,-57))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}