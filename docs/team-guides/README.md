# Team 4050 Swerve Drive Documentation Suite

Welcome to the comprehensive documentation for our CTRE Phoenix 6 swerve drive system! This documentation is designed to help our team members understand, tune, troubleshoot, and optimize our robot's drivetrain.

## � New to Swerve? Start Here!

**[📖 Getting Started Guide: From Generated Code to Driving Robot](./0-getting-started-guide.md)**

This blog-style tutorial walks you through the complete journey from Phoenix Tuner X generated code to a competition-ready swerve drive. Perfect for:

- First-time swerve developers
- Teams wanting a practical, step-by-step walkthrough
- Quick onboarding of new programmers
- Understanding the big picture before diving into details

**Start here if you:** Just generated code from Tuner X and want to know what to do next!

---

## �📚 Documentation Overview

This suite consists of six comprehensive guides, each focusing on a specific aspect of swerve drive development:

### 0️⃣ [Getting Started Guide](./0-getting-started-guide.md) ⭐ NEW!

**Purpose:** Practical walkthrough from generated code to operational robot

**Topics Covered:**

- First deploy and basic driving setup
- Hardware verification and calibration
- Quick tuning for immediate results
- Setting up autonomous with PathPlanner
- Adding telemetry and logging
- Pre-competition checklist
- Common gotchas and fixes

**When to Use:**

- You just generated code and need to know what's next
- Onboarding new programmers
- Quick reference for the essential steps
- Preparing for first practice or competition

---

### 1️⃣ [System Architecture](./1-system-architecture.md)

**Purpose:** Understand the complete structure of the Phoenix swerve system and how components interact

**Topics Covered:**

- Complete system architecture diagram
- Component breakdown (TunerConstants, SwerveDrivetrain, SwerveRequest API)
- Code architecture and flow
- Configuration deep dive
- Integration with WPILib and PathPlanner
- Best practices and team workflows

**When to Use:**

- Onboarding new team members
- Planning new features
- Understanding how to modify existing code
- Debugging integration issues
- Preparing for major code refactors

---

### 2️⃣ [Tuning Swerve Drive](./2-tuning-swerve-drive.md)

**Purpose:** Learn how to tune PID and feedforward gains for optimal swerve performance

**Topics Covered:**

- High-level control theory concepts
- Understanding steer, drive, and heading controllers
- Step-by-step tuning procedures (5 phases)
- SysId characterization
- Advanced tuning topics
- Validation and testing procedures

**When to Use:**

- Initial robot setup
- After mechanical changes (wheels, gearing, etc.)
- When experiencing control issues (oscillation, sluggish response)
- After firmware updates

---

### 3️⃣ [Autonomous Routines](./3-autonomous-routines.md)

**Purpose:** Build effective autonomous routines using PathPlanner and Phoenix swerve

**Topics Covered:**

- Autonomous fundamentals (coordinate systems, odometry)
- PathPlanner integration and configuration
- Building your first autonomous
- Advanced techniques (event markers, dynamic paths)
- Competition strategy
- Debugging autonomous
- Route templates

**When to Use:**

- Creating autonomous routines
- Optimizing autonomous for competition
- Learning path planning concepts
- Implementing complex multi-action sequences
- Debugging autonomous failures

---

### 4️⃣ [Optimization & Next Steps](./4-optimization-next-steps.md)

**Purpose:** Take your drivetrain to the next level with advanced features and optimizations

**Topics Covered:**

- Vision integration with AprilTags (PhotonVision)
- Advanced control features (heading lock, snap to angles, speed modes)
- Performance optimization (CANivore, threading, logging)
- Software architecture improvements
- Competition-ready features
- Next-level capabilities (obstacle avoidance, auto-alignment)
- Prioritized implementation roadmap

**When to Use:**

- After basic swerve is working reliably
- Planning season roadmap
- Adding vision-based pose estimation
- Implementing advanced driver assistance features
- Preparing for high-level competition

---

### 5️⃣ [Troubleshooting Guide](./5-troubleshooting.md)

**Purpose:** Diagnose and fix common (and uncommon) swerve drive issues

**Topics Covered:**

- Systematic debugging approach
- Issues organized by symptom
- Code-specific problems
- Simulation vs. real robot differences
- Emergency procedures
- Tools and techniques

**When to Use:**

- Robot not moving or moving erratically
- Specific modules misbehaving
- CAN bus errors
- Performance problems
- Pre-competition issues
- During matches when quick fixes needed

---

## 🚀 Getting Started

### For First-Time Swerve Setup:

**👉 Start with [Getting Started Guide](./0-getting-started-guide.md)** - This practical walkthrough takes you from Phoenix Tuner X generated code all the way to a competition-ready robot. Follow it step-by-step!

### For New Team Members:

1. Start with **[Getting Started Guide](./0-getting-started-guide.md)** for the practical big picture
2. Read **[System Architecture](./1-system-architecture.md)** to understand how everything fits together
3. Skim **[Tuning Guide](./2-tuning-swerve-drive.md)** high-level sections for control theory background
4. Keep **[Troubleshooting Guide](./5-troubleshooting.md)** bookmarked for when issues arise

### For Developers Adding Features:

1. Review **[System Architecture](./1-system-architecture.md)** → "Integration Points" and "Code Architecture"
2. Check **[Optimization Guide](./4-optimization-next-steps.md)** for implementation examples
3. Follow best practices from **[System Architecture](./1-system-architecture.md)** → "Best Practices"

### For Competition Preparation:

1. Complete **[Getting Started Guide](./0-getting-started-guide.md)** → "Pre-Competition Checklist"
2. Use **[Tuning Guide](./2-tuning-swerve-drive.md)** → "Validation Tests" checklist
3. Review **[Autonomous Guide](./3-autonomous-routines.md)** → "Competition Strategy"
4. Implement features from **[Optimization Guide](./4-optimization-next-steps.md)** → "Competition-Ready Features"
5. Have **[Troubleshooting Guide](./5-troubleshooting.md)** → "Emergency Procedures" ready

---

## 📋 Quick Reference

### Common Tasks

#### Task: Robot oscillates when driving

→ **[Troubleshooting](./5-troubleshooting.md)** → Category 2: Erratic Movement
→ **[Tuning Guide](./2-tuning-swerve-drive.md)** → Phase 3: Steer Motor PID

#### Task: Create new autonomous routine

→ **[Autonomous Guide](./3-autonomous-routines.md)** → Building Your First Auto
→ **[Autonomous Guide](./3-autonomous-routines.md)** → Autonomous Routine Templates

#### Task: Add vision-based pose estimation

→ **[Optimization Guide](./4-optimization-next-steps.md)** → Vision Integration
→ Implementation code examples included

#### Task: Understand TunerConstants.java

→ **[System Architecture](./1-system-architecture.md)** → Component Breakdown → TunerConstants

#### Task: Robot drives in wrong direction

→ **[Troubleshooting](./5-troubleshooting.md)** → Category 3: Inaccurate Movement
→ Check module inversions and encoder offsets

#### Task: Improve path following accuracy

→ **[Autonomous Guide](./3-autonomous-routines.md)** → Debugging Autonomous
→ **[Tuning Guide](./2-tuning-swerve-drive.md)** → Phase 2 & 3 (drive/steer tuning)
→ **[Optimization Guide](./4-optimization-next-steps.md)** → Performance Optimization

---

## 🛠️ Our Robot Configuration

### Hardware

- **Drive Motors:** 4x Talon FX (Kraken X60 or Falcon 500)
- **Steer Motors:** 4x Talon FX
- **Encoders:** 4x CANcoder (absolute position)
- **IMU:** Pigeon 2.0
- **CAN Bus:** roboRIO (standard) or CANivore (Pro)

### Software Stack

- **Framework:** WPILib Command-Based
- **Motor Control:** CTRE Phoenix 6
- **Swerve API:** Phoenix SwerveDrivetrain
- **Path Planning:** PathPlanner
- **Vision:** PhotonVision (planned)
- **Logging:** SignalLogger / AdvantageKit (optional)

### Key Files

```
src/main/java/frc/robot/
├── generated/
│   └── TunerConstants.java          [Phoenix-generated constants]
├── subsystems/
│   └── CommandSwerveDrivetrain.java [Our swerve subsystem]
├── commands/
│   └── auto/                        [Autonomous commands]
├── Constants.java                   [Team constants]
└── RobotContainer.java              [Command bindings]
```

---

## 📊 Tuning Status Tracker

Use this to track tuning progress:

```
□ Phase 1: Initial Setup & Validation
  □ Hardware verified
  □ Encoder offsets calibrated
  □ Module inversions correct

□ Phase 2: Drive Motor Tuning
  □ SysId characterization completed
  □ kV and kS set
  □ kP tuned for velocity tracking

□ Phase 3: Steer Motor Tuning
  □ SysId characterization completed
  □ kP tuned for position control
  □ kD added for damping
  □ kI determined (if needed)

□ Phase 4: Heading Controller
  □ SysId rotation characterization
  □ Heading kP tuned
  □ Heading kD added

□ Phase 5: Limits and Validation
  □ Slip current determined
  □ Max speed validated
  □ All validation tests passed

Current Gains (Record Date: ________):
Drive  - kP: _____ kI: _____ kD: _____ kS: _____ kV: _____
Steer  - kP: _____ kI: _____ kD: _____ kS: _____ kV: _____ kA: _____
Heading- kP: _____ kI: _____ kD: _____
```

---

## 🎯 Season Goals Roadmap

### Phase 1: Basic Swerve (Weeks 1-2) ✓

- [x] Deploy generated code
- [ ] Complete tuning (all 5 phases)
- [ ] Validate reliable operation
- [ ] Driver training

### Phase 2: Autonomous (Weeks 3-4)

- [ ] PathPlanner setup
- [ ] Basic mobility auto
- [ ] 2-piece autonomous
- [ ] Competition-ready autos

### Phase 3: Vision Integration (Weeks 5-6)

- [ ] Install PhotonVision hardware
- [ ] Camera calibration
- [ ] Vision pose estimation
- [ ] Tuning and validation

### Phase 4: Advanced Features (Weeks 7-8)

- [ ] Heading lock implementation
- [ ] Snap-to-angle
- [ ] Speed modes
- [ ] Driver feedback system

### Phase 5: Competition Optimization (Ongoing)

- [ ] CANivore + Pro license (if budget)
- [ ] Logging optimization
- [ ] Match data tracking
- [ ] Pre-match check system
