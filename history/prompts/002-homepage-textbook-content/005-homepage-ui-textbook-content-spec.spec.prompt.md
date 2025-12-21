---
id: 005
title: Homepage UI and Textbook Content Spec
stage: spec
date: 2025-12-21
surface: agent
model: gemini
feature: 002-homepage-textbook-content
branch: main
user: User
command: /sp.specify SPEC NAME: Phase 1 — Spec 2: Homepage UI Design + Textbook Content Creation CONTEXT: This project is an AI-native textbook + RAG chatbot system. Phase 1 — Spec 2 focuses on: - Designing a simple, clean UI - Writing or extending textbook content Docusaurus project and configuration are assumed to already exist from Spec 1. Book content may already exist and should NOT be overwritten. OBJECTIVE: Design a minimal homepage UI and create or extend textbook documentation covering Physical AI & Humanoid Robotics. ⚠️ DEPLOYMENT REQUIREMENT: The frontend MUST be **100% Vercel-ready** and deployable on **Vercel without any modification**. The Docusaurus build must be fully static, with no server-side dependencies. SCOPE: This spec includes: - Simple homepage UI design - Writing or extending textbook content as documentation pages This spec EXCLUDES: - Chatbot UI implementation - Backend or RAG logic - Vectorization or embeddings UI DESIGN GUIDELINES: - UI must be simple and readable - No advanced or complex UI components required - Focus on typography and layout clarity - Mobile-friendly by default - Documentation-first design (Docusaurus native patterns) TEXTBOOK STRUCTURE: The textbook is organized into FOUR modules: MODULE 1: The Robotic Nervous System (ROS 2) Focus: Middleware for robot control Topics: - ROS 2 architecture overview - Nodes, Topics, and Services - Bridging Python agents to ROS controllers using rclpy - Understanding URDF (Unified Robot Description Format) for humanoid robots MODULE 2: The Digital Twin (Gazebo & Unity) Focus: Physics simulation and environment building Topics: - Physics simulation fundamentals - Gravity, collisions, and constraints in Gazebo - High-fidelity rendering and human-robot interaction using Unity - Sensor simulation: - LiDAR - Depth cameras - IMUs MODULE 3: The AI-Robot Brain (NVIDIA Isaac™) Focus: Advanced perception and training Topics: - NVIDIA Isaac Sim overview - Photorealistic simulation and synthetic data generation - Isaac ROS - Hardware-accelerated Visual SLAM (VSLAM) - Navigation using Nav2 - Path planning for bipedal humanoid movement MODULE 4: Vision-Language-Action (VLA) Focus: The convergence of LLMs and Robotics Topics: - Vision-Language-Action paradigm - Voice-to-Action pipelines - Using OpenAI Whisper for voice commands - Cognitive planning with LLMs - Translating natural language tasks into ROS 2 action sequences Capstone Project: - The Autonomous Humanoid - A simulated humanoid robot that: - Receives a voice command - Plans a navigation path - Avoids obstacles - Identifies objects using computer vision - Manipulates an object to complete a task CONTENT REQUIREMENTS: - Each chapter must be: - Detailed and technical - Minimum 150 lines in length - Content must include: - External reference links - Images or diagrams - Tables where appropriate - Writing style should be: - Structured - Educational - Suitable for advanced learners - Existing book content must NOT be overwritten - New content can only be added or extended
---

## Prompt

SPEC NAME: Phase 1 — Spec 2: Homepage UI Design + Textbook Content Creation CONTEXT: This project is an AI-native textbook + RAG chatbot system. Phase 1 — Spec 2 focuses on: - Designing a simple, clean UI - Writing or extending textbook content Docusaurus project and configuration are assumed to already exist from Spec 1. Book content may already exist and should NOT be overwritten. OBJECTIVE: Design a minimal homepage UI and create or extend textbook documentation covering Physical AI & Humanoid Robotics. ⚠️ DEPLOYMENT REQUIREMENT: The frontend MUST be **100% Vercel-ready** and deployable on **Vercel without any modification**. The Docusaurus build must be fully static, with no server-side dependencies. SCOPE: This spec includes: - Simple homepage UI design - Writing or extending textbook content as documentation pages This spec EXCLUDES: - Chatbot UI implementation - Backend or RAG logic - Vectorization or embeddings UI DESIGN GUIDELINES: - UI must be simple and readable - No advanced or complex UI components required - Focus on typography and layout clarity - Mobile-friendly by default - Documentation-first design (Docusaurus native patterns) TEXTBOOK STRUCTURE: The textbook is organized into FOUR modules: MODULE 1: The Robotic Nervous System (ROS 2) Focus: Middleware for robot control Topics: - ROS 2 architecture overview - Nodes, Topics, and Services - Bridging Python agents to ROS controllers using rclpy - Understanding URDF (Unified Robot Description Format) for humanoid robots MODULE 2: The Digital Twin (Gazebo & Unity) Focus: Physics simulation and environment building Topics: - Physics simulation fundamentals - Gravity, collisions, and constraints in Gazebo - High-fidelity rendering and human-robot interaction using Unity - Sensor simulation: - LiDAR - Depth cameras - IMUs MODULE 3: The AI-Robot Brain (NVIDIA Isaac™) Focus: Advanced perception and training Topics: - NVIDIA Isaac Sim overview - Photorealistic simulation and synthetic data generation - Isaac ROS - Hardware-accelerated Visual SLAM (VSLAM) - Navigation using Nav2 - Path planning for bipedal humanoid movement MODULE 4: Vision-Language-Action (VLA) Focus: The convergence of LLMs and Robotics Topics: - Vision-Language-Action paradigm - Voice-to-Action pipelines - Using OpenAI Whisper for voice commands - Cognitive planning with LLMs - Translating natural language tasks into ROS 2 action sequences Capstone Project: - The Autonomous Humanoid - A simulated humanoid robot that: - Receives a voice command - Plans a navigation path - Avoids obstacles - Identifies objects using computer vision - Manipulates an object to complete a task CONTENT REQUIREMENTS: - Each chapter must be: - Detailed and technical - Minimum 150 lines in length - Content must include: - External reference links - Images or diagrams - Tables where appropriate - Writing style should be: - Structured - Educational - Suitable for advanced learners - Existing book content must NOT be overwritten - New content can only be added or extended

## Response snapshot

The feature specification for "Phase 1 — Spec 2: Homepage UI Design + Textbook Content Creation" has been successfully generated and validated.

**Key Actions Performed**:
- Created the feature directory `specs/002-homepage-textbook-content/`.
- Generated `specs/002-homepage-textbook-content/spec.md` with detailed user scenarios, functional requirements, key entities, and measurable success criteria.
- Generated `specs/002-homepage-textbook-content/checklists/requirements.md` and performed a quality validation check, confirming all criteria were met and the spec is ready for planning.

**Generated Artifacts**:
*   `specs/002-homepage-textbook-content/spec.md`
*   `specs/002-homepage-textbook-content/checklists/requirements.md`

## Outcome

- ✅ Impact: A well-defined and validated feature specification (`spec.md`) has been created for "Phase 1 — Spec 2: Homepage UI Design + Textbook Content Creation", along with its corresponding quality checklist. This provides a clear foundation for the next planning phase.
- 🧪 Tests: The specification was validated against a quality checklist (`specs/002-homepage-textbook-content/checklists/requirements.md`), and all criteria passed.
- 📁 Files: `specs/002-homepage-textbook-content/spec.md` and `specs/002-homepage-textbook-content/checklists/requirements.md` were created/updated.
- 🔁 Next prompts: The specification is ready for the planning phase. The next step would typically involve running `/sp.plan` to create an implementation plan based on this specification.
- 🧠 Reflection: The agent successfully processed the natural language description into a structured specification, adhering to all guidelines, including the critical single-branch development rule. The validation process ensures high quality and readiness for subsequent development phases.

## Evaluation notes (flywheel)

- Failure modes observed: Initial misunderstanding regarding branch creation was corrected, highlighting the importance of strict adherence to constitutional rules.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
