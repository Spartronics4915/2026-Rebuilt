Java Version: JDK 17

FRC Season/Game: 2026 REBUILT

Dependencies: 
    - BLine-Lib
    - CTRE-Phoenix 6
    - libgrapplefrc
    - PathplannerLib
    - photonlib
    - ReduxLib
    - ThriftyLib
    - WPILib-New-Commands
    - WPILib

Robot Structure:
    Archetype: corner turreted shooter

    Mechanisms:
        - Intake pivot (moves the whole intake structure, pivot not linear slide) (kraken x44)
        - Intake wheels (picks up balls from the ground) (kraken x60)
        - Indexer (Spindexer, wheel spinning balls into feeder) (kraken x60)
        - Feeder wheels (wheels feeding balls into turret) (kraken x60)
        - Turret base (moves the shooter, hood, and turreted camera, changes the balls exit azimuth) (kraken x44)
        - Shooter (single flywheel shooter made to lanuch balls coming from the feeder into the hub or to pass balls to alliance side) (2 kraken x60)
        - hood (3D printed hood made to change the launch angle of the balls)
        - swerve drive (4 wheels each powered by their own x60 for drive and x44 for steer)

    Programming Goals:
        - Flexible autonomous routines made to work with any robots we may play with/against
        - Stable precise localization on the field using vision systems + wheel odometry
        - Shooting on the fly, this means adjusting hood, turret, and shooter setpoints automatically based on physics equations.
            - Shooting on the move, so the driver can spin and drive the chassis while the robot keeps perfect aim, allowing for maximum scoring potential
        - Correct tuning and control types for all moving mechanisms, for accurate and stable movement
        - Logging of lots of diagnositc info for debugging, tuning, and physical error management
        - Near 0 loop overruns after initial startup, as loop overruns can be catastrophic for the entire codebase
        - Localtion and time based state machine handling of relativant subsystems to releave workload off of driver

    Programming Structure:
        - We have a bunch of "Subsystems" which are each independent system for example each mechanism has a subsystem, vision also has one.
        - The subsystem are all controlled by two overarching systems called AutoAimController and Superstructure
            - AutoAimController handles the aiming functions of the robot using our AutoAim utility class, this orchestrates the turret, shooter, and hood mainly
            - Superstructure is what controls the state of the robot, it use zones on the field the determine the desired state of the robot, it also doesnt some other minor things
        - Everything is created in RobotContainer, and that is also where our controller bindings are defined
        - Constants is where all of our constants are stored for everything on the robot.
        - in our util folder is where we keep a bunch of utility classes that dont belong inside other files.

    Physical specifications:
        - The robot has a total of 4 cameras, a Limelight 4 on the turret, and 3 static Luma P1 cameras running Photon Vision
        - 140 pounds with bumpers and battery
        - The entire robot runs on one canivore from CTRE
        - The robot's name is Artemis


Current State:
    - The 2026 REBUILT offical season is over, and so we are now getting ready for offseason competitions
    - The robot won its first competition, was the 3rd alliances first pick in its 3rd competition, at districts it was a semi finalist as the 1st pick of the second alliance, it won the Milstein division at worlds as the 2nd pick of 1678 and 2481, and made it to the Einstein playoffs.

    Issues:
        - Currently on our sweeps in the neutral zone in auto after the first one the robot can seem to intake balls
        - We have a new vision system that is not well tuned
        - Loop overruns have plagued us all season, they have caused a lot of issues in the past
            - Currently we think the main culprit is vision, but it could very well be something else or a mix of many things
        - Brown outs at the end of intense matches, also when ferrying from opposite alliance side and driving our main breaker has popped before

    Main Concern:
        - Currently our main concern is the loop overruns as they are making it so the robot is basically unable to function even close to 30% of its potential.

    