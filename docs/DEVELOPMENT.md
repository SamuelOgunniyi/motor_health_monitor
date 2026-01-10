# Development Process

This project uses AI-assisted development tools (Cursor AI) for implementation assistance. All architectural decisions, design choices, and final code are the author's work.

This project also serves as a learning exercise to reinforce and deepen my understanding of less obvious C++ knowledge, the ROS 2 ecosystem, the robotics domain, and critical thinking about problem decomposition and solution design. However, it is still meant to be a functional, production-ready tool that users can deploy.

Monitoring tools are often underrated, yet they serve as perfect testbeds for testing concepts from different parts of the control loop. This tool acts as a safety guard to protect robots from damaging actions, reduce downtime, and ensure operator safety.

## Original Work

**Author's Contributions:**
- Concept and requirements specification
- Architectural decisions (factory pattern, generic fault detection, auto-subscription)
- Design philosophy (motor controller abstraction, two distinct monitoring processes)
- Real-world use case understanding

## AI-Assisted

**Implementation Help:**
- Code generation based on author's specifications
- Refactoring assistance (ROS 2 message types, lifecycle conversion)
- Bug fixes and code quality improvements

## Verification

All code was reviewed, understood, tested, and customized by the author before committing. See commit history for development process.

## Code Ownership

All code is the intellectual property of Samuel Ogunniyi. AI tools were used as development assistants only; all architectural decisions and final implementations represent the author's work.
