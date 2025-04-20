package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
                        .splineToConstantHeading(
                                new Vector2d(15, -23), 0,
                                new TranslationalVelConstraint(100),
                                new ProfileAccelConstraint(70, 200)
                        )
                        .splineToConstantHeading(
                                new Vector2d(50, -23), 0,
                                new TranslationalVelConstraint(100),
                                new ProfileAccelConstraint(70, 200)
                        )

                        .splineToConstantHeading(
                                new Vector2d(50, -31), -Math.PI / 2,
                                new TranslationalVelConstraint(100),
                                new ProfileAccelConstraint(70, 200)
                        )

                        .setReversed(true)
                        .splineToConstantHeading(
                                new Vector2d(5, -31), Math.PI,
                                new TranslationalVelConstraint(100),
                                new ProfileAccelConstraint(70, 200)
                        )

                        .setReversed(false)
                        .splineToConstantHeading(
                                new Vector2d(50, -31), 0,
                                new TranslationalVelConstraint(100),
                                new ProfileAccelConstraint(70, 200)
                        )
                        .splineToConstantHeading(
                                new Vector2d(50, -41), -Math.PI / 2,
                                new TranslationalVelConstraint(100),
                                new ProfileAccelConstraint(70, 200)
                        )

                        .setReversed(true)
                        .splineToConstantHeading(
                                new Vector2d(5, -41), Math.PI,
                                new TranslationalVelConstraint(100),
                                new ProfileAccelConstraint(70, 200)
                        )

                        .setReversed(false)
                        .splineToConstantHeading(
                                new Vector2d(50, -41), 0,
                                new TranslationalVelConstraint(100),
                                new ProfileAccelConstraint(70, 200)
                        )
                        .splineToConstantHeading(
                                new Vector2d(50, -46), -Math.PI / 2,
                                new TranslationalVelConstraint(100),
                                new ProfileAccelConstraint(70, 200)
                        )

                        .setReversed(true)
                        .splineToConstantHeading(
                                new Vector2d(5, -46), Math.PI,
                                new TranslationalVelConstraint(100),
                                new ProfileAccelConstraint(70, 200)
                        )
                .build());


                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}