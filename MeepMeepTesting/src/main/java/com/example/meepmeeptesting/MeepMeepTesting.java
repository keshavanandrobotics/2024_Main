package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                        .splineToConstantHeading(new Vector2d(16, -30), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(42, -30), 0)
                        .splineToConstantHeading(new Vector2d(42, -35), 0)
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(7, -35), 0)

                        .setReversed(false)

                        .splineToConstantHeading(new Vector2d(42, -35), 0)

                        .splineToConstantHeading(new Vector2d(42, -45), 0)

                        //

                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(7, -45), 0)

                        .setReversed(false)

                        .splineToConstantHeading(new Vector2d(42, -45), 0)

                        .splineToConstantHeading(new Vector2d(42, -53), 0)

                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(7, -53), 0)
                                .build());




                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}