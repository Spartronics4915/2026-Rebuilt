package com.spartronics4915.frc2026.autos;

import static com.spartronics4915.frc2026.autos.ComplexAutoChooser.AutoSegment.*;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.spartronics4915.frc2026.autos.ZoneTransition.TraversalMethod;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static com.spartronics4915.frc2026.autos.ComplexAutoChooser.AllowedTransitions.*;

public class ComplexAutoChooser {
    /**
     * Enum representing all the "steps" of an auto routine to be selected in elastic.
     * Each step contains the allowed step following it, which is used to determine which options to show in the UI after a step is selected.
     */
    public enum AutoSegment {
        L_TRENCH_TO_NEUTRAL("LT -> N", WITHIN_NEUTRAL),
        L_BUMP_TO_NEUTRAL("LB -> N", WITHIN_NEUTRAL),
        R_TRENCH_TO_NEUTRAL("RT -> N", WITHIN_NEUTRAL),
        R_BUMP_TO_NEUTRAL("RB -> N", WITHIN_NEUTRAL),
        L_TRENCH_TO_ALLIANCE("LT -> A", WITHIN_ALLIANCE),
        L_BUMP_TO_ALLIANCE("LB -> A", WITHIN_ALLIANCE),
        R_TRENCH_TO_ALLIANCE("RT -> A", WITHIN_ALLIANCE),
        R_BUMP_TO_ALLIANCE("RB -> A", WITHIN_ALLIANCE),
        DEPOT("-> D", WITHIN_ALLIANCE),
        OUTPOST("-> O", WITHIN_ALLIANCE),
        TOWER("-> T", NONE),
        UNUSED(" ", NONE);

        public final String userFacingName;
        private final AllowedTransitions allowedTransitions;

        AutoSegment(String userFacingName, AllowedTransitions allowedTransitions) {
            this.userFacingName = userFacingName;
            this.allowedTransitions = allowedTransitions;
        }

        public AutoSegment[] getAllowedTransitions() {
            return allowedTransitions.getAllowedSegments();
        }
    }

    /**
     * Enum used to shorten (and remove enum loop) of the AutoSegment enum.
     */
    public enum AllowedTransitions {
        WITHIN_NEUTRAL(() -> new AutoSegment[]{L_TRENCH_TO_ALLIANCE, L_BUMP_TO_ALLIANCE, R_TRENCH_TO_ALLIANCE, R_BUMP_TO_ALLIANCE}),
        WITHIN_ALLIANCE(() -> new AutoSegment[]{L_TRENCH_TO_NEUTRAL, L_BUMP_TO_NEUTRAL, R_TRENCH_TO_NEUTRAL, R_BUMP_TO_NEUTRAL, DEPOT, OUTPOST, TOWER}),
        NONE(() -> new AutoSegment[]{});

        private final Supplier<AutoSegment[]> allowedSegmentsSupplier;
        private AutoSegment[] allowedSegments;
        
        AllowedTransitions(Supplier<AutoSegment[]> allowedSegmentsSupplier) {
            this.allowedSegmentsSupplier = allowedSegmentsSupplier;
        }

        public AutoSegment[] getAllowedSegments() {
            if (allowedSegments == null) {
                allowedSegments = allowedSegmentsSupplier.get();
            }
            return allowedSegments;
        }
    }

    private final ZoneTransition transitionFactory;
    private final DriveToPOI POIFactory;
    private final NeutralZoneAutos neutralZoneFactory;

    private AutoSegment[] selectedSegments;
    private SendableChooser<AutoSegment>[] segmentChoosers;

    @SuppressWarnings("unchecked")
    /**
     * Constructor for ComplexAutoChooser. Initializes the SendableChoosers for each auto segment step, and sets up the default options and onChange listeners.
     * @param transitionFactory Factory for generating zone transition commands.
     * @param POIfactory Factory for generating drive-to-POI commands.
     * @param neutralZoneFactory Factory for generating neutral zone commands.
     * @param maxSegments Maximum number of auto segments.
     */
    public ComplexAutoChooser(ZoneTransition transitionFactory, DriveToPOI POIFactory, NeutralZoneAutos neutralZoneFactory, int maxSegments) {
        this.transitionFactory = transitionFactory;
        this.POIFactory = POIFactory;
        this.neutralZoneFactory = neutralZoneFactory;

        this.selectedSegments = new AutoSegment[maxSegments];
        this.segmentChoosers = (SendableChooser<AutoSegment>[]) new SendableChooser[maxSegments];
        for (int i = 0; i < maxSegments; i++) {
            selectedSegments[i] = UNUSED;
        }
        resolveSteps();
    }

    /**
     * Resolves the options for each auto segment to make sure only valid options are shown based on the currently selected segments. Should be called whenever a segment is changed.
     */
    public void resolveSteps() {
        AutoSegment lastSegment = L_TRENCH_TO_ALLIANCE; // Default starting segment, since the robot starts on alliance side of the trench
        if (segmentChoosers[0] != null) {
            for (int i = 0; i < selectedSegments.length; i++) {
                segmentChoosers[i].close();
            }
        }

        for (int i = 0; i < selectedSegments.length; i++) {
            SendableChooser<AutoSegment> segment = new SendableChooser<>();

            if (lastSegment.getAllowedTransitions().length == 0) {
                selectedSegments[i] = UNUSED;
            } else {
                for (AutoSegment option : lastSegment.getAllowedTransitions()) {
                    if (option == lastSegment) continue;
                    segment.addOption(option.userFacingName, option);
                }

                segment.setDefaultOption(UNUSED.userFacingName, UNUSED);

                final int j = i;
                segment.onChange((selected) -> {
                    if (selectedSegments[j] != selected) {
                        selectedSegments[j] = selected;
                        resolveSteps();
                    }
                });
            }

            SmartDashboard.putData("Auto Chooser/Step " + i, segment);
            
            segmentChoosers[i] = segment;
            lastSegment = selectedSegments[i] != null ? selectedSegments[i] : UNUSED;
        }
    }

    private Command addNeutralZoneCommand(boolean inRight, boolean outRight) {
        if (inRight == outRight) {
            return neutralZoneFactory.generateQuadrantCommand(inRight);
        } else {
            return neutralZoneFactory.generateHalfCommand(inRight);
        }
    }

    /**
     * Generates the auto command based on the currently selected segments.
     * @return The command representing the selected autonomous routine.
     */
    public Command getAuto() {
        ArrayList<Command> commands = new ArrayList<>();
        boolean right = false;
        for (int i = 0; i < selectedSegments.length; i++) {
            AutoSegment segment = selectedSegments[i];

            if (segment == UNUSED) {
                break;
            }

            switch (segment) {
                case L_TRENCH_TO_NEUTRAL:
                    right = false;
                    commands.add(transitionFactory.generateCommand(TraversalMethod.LEFT_TRENCH, true));
                    break;
                case L_BUMP_TO_NEUTRAL:
                    right = false;
                    commands.add(transitionFactory.generateCommand(TraversalMethod.LEFT_BUMP, true));
                    break;
                case R_TRENCH_TO_NEUTRAL:
                    right = true;
                    commands.add(transitionFactory.generateCommand(TraversalMethod.RIGHT_TRENCH, true));
                    break;
                case R_BUMP_TO_NEUTRAL:
                    right = true;
                    commands.add(transitionFactory.generateCommand(TraversalMethod.RIGHT_BUMP, true));
                    break;
                case L_TRENCH_TO_ALLIANCE:
                    commands.add(addNeutralZoneCommand(right, false));
                    commands.add(transitionFactory.generateCommand(TraversalMethod.LEFT_TRENCH, false));
                    break;
                case L_BUMP_TO_ALLIANCE:
                    commands.add(addNeutralZoneCommand(right, false));
                    commands.add(transitionFactory.generateCommand(TraversalMethod.LEFT_BUMP, false));
                    break;
                case R_TRENCH_TO_ALLIANCE:
                    commands.add(addNeutralZoneCommand(right, true));
                    commands.add(transitionFactory.generateCommand(TraversalMethod.RIGHT_TRENCH, false));
                    break;
                case R_BUMP_TO_ALLIANCE:
                    commands.add(addNeutralZoneCommand(right, true));
                    commands.add(transitionFactory.generateCommand(TraversalMethod.RIGHT_BUMP, false));
                    break;
                case DEPOT:
                    commands.add(POIFactory.generateCommand(DriveToPOI.POI.DEPOT));
                    break;
                case OUTPOST:
                    commands.add(POIFactory.generateCommand(DriveToPOI.POI.OUTPOST));
                    break;
                case TOWER:
                    commands.add(POIFactory.generateCommand(DriveToPOI.POI.TOWER));
                    break;
                case UNUSED:
                    System.out.println("Chat what are we doing?");
                    break;
            }
        }

        Command[] commandsArray = new Command[commands.size()];
        commands.toArray(commandsArray);

        return new SequentialCommandGroup(commandsArray);
    }
}
