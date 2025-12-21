---

description: "Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts."
---

# Tasks: Phase 1 — Spec 2: Homepage UI Design + Textbook Content Creation

**Input**: Design documents from `/specs/002-homepage-textbook-content/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths shown below are relative to the project root.

## Phase 1: Setup (Initial Review)

**Purpose**: Verify the Docusaurus environment is ready for UI and content work. This is a quick check, assuming prior configuration from Spec 1.

- [X] T001 Verify the Docusaurus project in `frontend/` is correctly configured and `npm install` has been run. (Check `frontend/package.json` and `frontend/docusaurus.config.ts`)

## Phase 2: Foundational (No New Foundational Tasks)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented. Assumed complete from Spec 1.

## Phase 3: User Story 1 - Homepage Display (Priority: P1) 🎯 MVP

**Goal**: As a user, I need to see a clean and readable homepage so that I can easily understand the project's purpose and navigate to the textbook content.
**Independent Test**: Can be fully tested by accessing the site's root URL and observing a well-structured, visually appealing homepage that clearly guides users to the textbook.

### Implementation for User Story 1

- [X] T002 [P] [US1] Customize `frontend/src/pages/index.tsx` to display a minimal, readable homepage UI, focusing on typography and layout clarity.
- [X] T003 [P] [US1] Ensure the homepage UI is mobile-friendly by testing Docusaurus's default responsiveness and making minor adjustments in `frontend/src/css/custom.css` if necessary, following documentation-first design.
- [X] T004 [P] [US1] Verify that the docs pages UI visually aligns with the Navbar and ensures consistent Light, Dark, and System theme modes across the homepage.

## Phase 4: User Story 2 - Textbook Content Access (Priority: P1)

**Goal**: As a learner, I need to access comprehensive and well-structured textbook content so that I can learn about Physical AI & Humanoid Robotics.
**Independent Test**: Can be fully tested by navigating through all modules and chapters, verifying that content is displayed, formatted correctly, and meets specified length/detail requirements.

### Implementation for User Story 2

- [X] T005 [P] [US2] Review existing chapters in `frontend/docs/` for content gaps and identify opportunities for extension or new chapter creation.
- [X] T006 [P] [US2] Create or extend textbook content for **Module 1 (ROS 2)**, ensuring each chapter is detailed, technical, at least 150 lines, and includes external links, images, diagrams, and tables where appropriate (files in `frontend/docs/Module-1-ROS 2/`).
- [X] T007 [P] [US2] Create or extend textbook content for **Module 2 (Gazebo & Unity)**, ensuring each chapter is detailed, technical, at least 150 lines, and includes external links, images, diagrams, and tables where appropriate (files in `frontend/docs/Module-2-Gazebo-Unity/`).
- [X] T008 [P] [US2] Create or extend textbook content for **Module 3 (NVIDIA Isaac™)**, ensuring each chapter is detailed, technical, at least 150 lines, and includes external links, images, diagrams, and tables where appropriate (files in `frontend/docs/Module-3-ISAAC/`).
- [X] T009 [P] [US2] Create or extend textbook content for **Module 4 (Vision-Language-Action)**, ensuring each chapter is detailed, technical, at least 150 lines, and includes external links, images, diagrams, and tables where appropriate (files in `frontend/docs/Module-4-VLA/`).
- [X] T010 [P] [US2] Ensure all content adheres to a structured, educational writing style suitable for advanced learners, and that existing content is not overwritten.
- [X] T011 [P] [US2] Verify that documentation pages use full viewport height for both sidebars and visually align with the Navbar, potentially adjusting `frontend/src/css/custom.css`.
- [X] T012 [P] [US2] Check Light, Dark, and System theme modes apply consistently across Documentation pages and Sidebars, without theme or style mismatches.

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Final verification of deployment readiness and overall site quality.

- [X] T013 Ensure the Docusaurus build is fully static, with no server-side dependencies, by running `npm run build` in `frontend/` and inspecting the output.
- [X] T014 Verify the frontend is 100% Vercel-ready and deployable on Vercel without any modification. (This involves a trial deployment or detailed check of `vercel.json` if present).
- [X] T015 Perform a comprehensive visual inspection of the entire site (homepage, documentation pages) across desktop and mobile viewports to ensure clean typography, proper spacing, and overall mobile responsiveness.

---
## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately.
-   **Foundational (Phase 2)**: No new tasks. Assumed complete from Spec 1.
-   **User Stories (Phase 3+)**: All depend on Setup phase completion.
    -   User stories can then proceed in parallel (if staffed)
    -   Or sequentially in priority order (P1 -> P1 -> ...)
-   **Polish (Final Phase)**: Depends on all desired user stories being complete.

### User Story Dependencies

-   **User Story 1 (P1)**: Can start after Setup (Phase 1) - No dependencies on other stories.
-   **User Story 2 (P1)**: Can start after Setup (Phase 1) - No dependencies on other stories.

### Within Each User Story

-   Implementation tasks are ordered logically to minimize dependencies.
-   Story complete before moving to next priority.

### Parallel Opportunities

-   **User Story 1**: Tasks T002, T003, T004 can be done in parallel.
-   **User Story 2**: Tasks T006-T009 (content creation for each module) can be done in parallel. Tasks T005, T010, T011, T012 can also be done in parallel or alongside content creation.

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 3: User Story 1.
3.  **STOP and VALIDATE**: Test User Story 1 independently by running `npm start` and verifying the homepage.
4.  Deploy/demo if ready.

### Incremental Delivery

1.  Complete Setup → Foundation ready.
2.  Add User Story 1 → Test independently → Deploy/Demo (MVP!).
3.  Add User Story 2 (module by module) → Test independently → Deploy/Demo.

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup together.
2.  Once Setup is done:
    -   Developer A: User Story 1 tasks.
    -   Developer B: User Story 2 content creation for specific modules.
    -   Developer C: User Story 2 theming/layout verification tasks.

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Verify tests fail before implementing
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence