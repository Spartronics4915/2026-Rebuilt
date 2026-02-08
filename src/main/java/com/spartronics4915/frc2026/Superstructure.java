package com.spartronics4915.frc2026;

import com.spartronics4915.frc2026.subsystems.mechanisms.path.IntakeSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.path.HoodSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.path.ShooterSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.path.SpindexerSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.path.TurretSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.ClimberSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.PivotSubsystem;

public class Superstructure {

    private final HoodSubsystem hood;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final SpindexerSubsystem spindexer;
    private final TurretSubsystem turret;
    private final ClimberSubsystem climber;
    private final PivotSubsystem pivot;

    public Superstructure() {
        this.hood = new HoodSubsystem();
        this.intake = new IntakeSubsystem();
        this.shooter = new ShooterSubsystem();
        this.spindexer = new SpindexerSubsystem();
        this.turret = new TurretSubsystem();
        this.climber = new ClimberSubsystem();
        this.pivot = new PivotSubsystem();
    }
    
    public enum SuperState {
        TRAVERSAL,
        SHOOTING,
        TRENCH,
        PRE_CLIMB,
        ACTIVE_CLIMB,
        STOWED
    }

}
