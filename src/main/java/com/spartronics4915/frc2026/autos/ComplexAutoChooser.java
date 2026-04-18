package com.spartronics4915.frc2026.autos;

import static com.spartronics4915.frc2026.autos.ComplexAutoChooser.AutoSegment.*;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.function.Supplier;

import com.spartronics4915.frc2026.autos.NeutralZoneAutos.IntakeShift;
import com.spartronics4915.frc2026.autos.ZoneTransition.TraversalMethod;
import com.spartronics4915.frc2026.subsystems.control.Superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.defaultShootWaitTime;
import static com.spartronics4915.frc2026.autos.ComplexAutoChooser.AllowedTransitions.*;

public class ComplexAutoChooser {
    /**
     * Enum representing all the "steps" of an auto routine to be selected in elastic.
     * Each step contains the allowed step following it, which is used to determine which options to show in the UI after a step is selected.
     */
    public enum AutoSegment {
        L_TRENCH_TO_NEUTRAL("LT -> N", READY_TO_INTAKE),
        L_BUMP_TO_NEUTRAL("LB -> N", READY_TO_INTAKE),
        R_TRENCH_TO_NEUTRAL("RT -> N", READY_TO_INTAKE),
        R_BUMP_TO_NEUTRAL("RB -> N", READY_TO_INTAKE),

        INTAKE_QUARTER("I Quarter", DIST_SELECT),
        INTAKE_HALF("I Half", DIST_SELECT),
        INTAKE_HAIRPIN("I Quarter (w/ Hairpin)", DIST_SELECT),
        INTAKE_INVERTED_QUARTER("I Inverted Quarter", DIST_SELECT),
        INTAKE_MIDDLE("I Middle", WITHIN_NEUTRAL),

        INTAKE_CLOSE("Intake Close", WITHIN_NEUTRAL),
        INTAKE_NORMAL("Intake Normal", WITHIN_NEUTRAL),
        INTAKE_FAR("Intake Far", WITHIN_NEUTRAL),

        L_TRENCH_TO_ALLIANCE("LT -> A", WITHIN_ALLIANCE),
        L_BUMP_TO_ALLIANCE("LB -> A", WITHIN_ALLIANCE),
        R_TRENCH_TO_ALLIANCE("RT -> A", WITHIN_ALLIANCE),
        R_BUMP_TO_ALLIANCE("RB -> A", WITHIN_ALLIANCE),

        DEPOT("-> D", WITHIN_ALLIANCE),
        OUTPOST("-> O", WITHIN_ALLIANCE),
        TOWER("-> T", NONE),
        PAUSE("P", WITHIN_ALLIANCE),
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
        READY_TO_INTAKE(() -> new AutoSegment[]{INTAKE_QUARTER, INTAKE_HALF, INTAKE_HAIRPIN, INTAKE_MIDDLE}),
        DIST_SELECT(() -> new AutoSegment[]{INTAKE_CLOSE, INTAKE_NORMAL, INTAKE_FAR}),
        WITHIN_NEUTRAL(() -> new AutoSegment[]{L_TRENCH_TO_ALLIANCE, L_BUMP_TO_ALLIANCE, R_TRENCH_TO_ALLIANCE, R_BUMP_TO_ALLIANCE}),
        WITHIN_ALLIANCE(() -> new AutoSegment[]{L_TRENCH_TO_NEUTRAL, L_BUMP_TO_NEUTRAL, R_TRENCH_TO_NEUTRAL, R_BUMP_TO_NEUTRAL, DEPOT, OUTPOST, TOWER, PAUSE}),
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
    private final PreAlignment preAlignmentFactory;
    private final Superstructure superstructure;

    private double shootWaitTime;

    private AutoSegment[] selectedSegments;
    private SendableChooser<AutoSegment>[] segmentChoosers;
    private AutoSegment[] segmentSources;

    @SuppressWarnings("unchecked")
    /**
     * Constructor for ComplexAutoChooser. Initializes the SendableChoosers for each auto segment step, and sets up the default options and onChange listeners.
     * @param transitionFactory Factory for generating zone transition commands.
     * @param POIfactory Factory for generating drive-to-POI commands.
     * @param neutralZoneFactory Factory for generating neutral zone commands.
     * @param preAlignmentFactory Factory for generating pre-alignment (OP Tech) commands.
     * @param superstructure Superstructure subsystem.
     * @param maxSegments Maximum number of auto segments.
     */
    public ComplexAutoChooser(ZoneTransition transitionFactory, DriveToPOI POIFactory, NeutralZoneAutos neutralZoneFactory, PreAlignment preAlignmentFactory, Superstructure superstructure, int maxSegments) {
        this.transitionFactory = transitionFactory;
        this.POIFactory = POIFactory;
        this.neutralZoneFactory = neutralZoneFactory;
        this.preAlignmentFactory = preAlignmentFactory;
        this.superstructure = superstructure;

        this.selectedSegments = new AutoSegment[maxSegments];
        this.segmentChoosers = (SendableChooser<AutoSegment>[]) new SendableChooser[maxSegments];
        this.segmentSources = new AutoSegment[maxSegments];
        for (int i = 0; i < maxSegments; i++) {
            selectedSegments[i] = UNUSED;
        }
        resolveSteps();

        shootWaitTime = SmartDashboard.getNumber("Auto Chooser/Shoot Wait Time", defaultShootWaitTime);

        SmartDashboard.putNumber("Auto Chooser/Shoot Wait Time", shootWaitTime);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard/Auto Chooser");
        table.addListener("Shoot Wait Time", EnumSet.of(NetworkTableEvent.Kind.kValueAll), (filler, key, event) -> {
            shootWaitTime = event.valueData.value.getDouble();
        });
    }

    /**
     * Resolves the options for each auto segment to make sure only valid options are shown based on the currently selected segments. Should be called whenever a segment is changed.
     */
    public void resolveSteps() {
        AutoSegment lastSegment = L_TRENCH_TO_ALLIANCE; // Default starting segment, since the robot starts on alliance side of the trench

        for (int i = 0; i < selectedSegments.length; i++) {
            if (segmentSources[i] == lastSegment) {
                lastSegment = selectedSegments[i] != null ? selectedSegments[i] : UNUSED;
                continue;
            }

            if (segmentChoosers[i] != null) {
                segmentChoosers[i].close();
            }

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
            segmentSources[i] = lastSegment;
            lastSegment = selectedSegments[i] != null ? selectedSegments[i] : UNUSED;
        }
    }

    private boolean isRight(AutoSegment segment) {
        // Triggers with RT -> N, RB -> N, RT -> A, and RB -> A, the rest are "left"
        return segment.userFacingName.charAt(0) == 'R';
    }

    private AutoSegment getNextNonPauseSegment(int currentIndex) {
        for (int i = currentIndex + 1; i < selectedSegments.length; i++) {
            if (selectedSegments[i] != PAUSE) {
                return selectedSegments[i] != null ? selectedSegments[i] : UNUSED;
            }
        }
        return UNUSED;
    }

    /**
     * Generates the auto command based on the currently selected segments.
     * @return The command representing the selected autonomous routine.
     */
    public Command getAuto() {
        ArrayList<Command> commands = new ArrayList<>();
        AutoSegment prevSegment = UNUSED;
        for (int i = 0; i < selectedSegments.length; i++) {
            AutoSegment currentSegment = selectedSegments[i];
            AutoSegment futureSegment = (i + 1 < selectedSegments.length) ? selectedSegments[i + 1] : UNUSED;
            IntakeShift intakeShift = NeutralZoneAutos.convertToIntakeShift(futureSegment);

            if (currentSegment == UNUSED) {
                break;
            }
            
            boolean useStartingConstraints = (i == 1) && (prevSegment == L_TRENCH_TO_NEUTRAL || prevSegment == R_TRENCH_TO_NEUTRAL);

            switch (currentSegment) {
                case L_TRENCH_TO_NEUTRAL:
                    commands.add(
                        prevSegment == UNUSED 
                            ? transitionFactory.generateStartingTrenchCommand(false) 
                            : transitionFactory.generateCommand(TraversalMethod.LEFT_TRENCH, true)
                    );
                    break;
                case L_BUMP_TO_NEUTRAL:
                    commands.add(transitionFactory.generateCommand(TraversalMethod.LEFT_BUMP, true));
                    break;
                case R_TRENCH_TO_NEUTRAL:
                    commands.add(
                        prevSegment == UNUSED 
                            ? transitionFactory.generateStartingTrenchCommand(true) 
                            : transitionFactory.generateCommand(TraversalMethod.RIGHT_TRENCH, true)
                    );
                    break;
                case R_BUMP_TO_NEUTRAL:
                    commands.add(transitionFactory.generateCommand(TraversalMethod.RIGHT_BUMP, true));
                    break;

                case INTAKE_CLOSE:
                case INTAKE_NORMAL:
                case INTAKE_FAR:
                    break;

                case INTAKE_QUARTER:
                    commands.add(neutralZoneFactory.generateQuadrantCommand(isRight(prevSegment), intakeShift));
                    break;
                case INTAKE_HALF:
                    commands.add(neutralZoneFactory.generateHalfCommand(isRight(prevSegment), intakeShift));
                    break;
                case INTAKE_INVERTED_QUARTER:
                    AutoSegment moreFutureSegment = (i + 2 < selectedSegments.length) ? selectedSegments[i + 2] : UNUSED;
                    commands.add(neutralZoneFactory.generateInvertedQuadrantCommand(isRight(moreFutureSegment), intakeShift));
                    break;
                case INTAKE_HAIRPIN:
                    commands.add(neutralZoneFactory.generateHairpinCommand(isRight(prevSegment), intakeShift));
                    break;

                case INTAKE_MIDDLE:
                    commands.add(neutralZoneFactory.generateMiddleCommand());
                    break;

                case L_TRENCH_TO_ALLIANCE:
                    commands.add(transitionFactory.generateCommand(TraversalMethod.LEFT_TRENCH, false, futureSegment != PAUSE && futureSegment != UNUSED, getNextNonPauseSegment(i) == L_TRENCH_TO_NEUTRAL));
                    break;
                case L_BUMP_TO_ALLIANCE:
                    commands.add(transitionFactory.generateCommand(TraversalMethod.LEFT_BUMP, futureSegment != PAUSE && futureSegment != UNUSED));
                    break;
                case R_TRENCH_TO_ALLIANCE:
                    commands.add(transitionFactory.generateCommand(TraversalMethod.RIGHT_TRENCH, false, futureSegment != PAUSE && futureSegment != UNUSED, getNextNonPauseSegment(i) == R_TRENCH_TO_NEUTRAL));
                    break;
                case R_BUMP_TO_ALLIANCE:
                    commands.add(transitionFactory.generateCommand(TraversalMethod.RIGHT_BUMP, futureSegment != PAUSE && futureSegment != UNUSED));
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
                case PAUSE:
                    commands.add(
                        // This is very hard to read, but it runs preAlignment along with a bunch of wait conditions.
                        // The max limit is the shootWaitTime set by the user, otherwise it'll end earlier if no balls are detected after 0.3 seconds of waiting
                        Commands.deadline(
                            Commands.race(
                                Commands.waitSeconds(shootWaitTime)
                                // Commands.sequence(
                                //    Commands.waitUntil(() -> superstructure.isBallDetectedDebounced())
                                //    Commands.waitSeconds(0.25),
                                //    Commands.waitUntil(() -> !superstructure.isBallDetectedDebounced())
                                // )
                            ),
                            preAlignmentFactory.generateCommand(prevSegment, futureSegment)
                        )
                    );
                    break;
                case UNUSED:
                    System.out.println("Chat what are we doing?");
                    break;
            }

            prevSegment = currentSegment;
        }

        Command[] commandsArray = new Command[commands.size()];
        commands.toArray(commandsArray);

        return new SequentialCommandGroup(commandsArray);
    }
}
