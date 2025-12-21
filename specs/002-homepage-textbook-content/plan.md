# Implementation Plan: Phase 1 — Spec 2: Homepage UI Design + Textbook Content Creation

**Branch**: `main` | **Date**: 2025-12-21 | **Spec**: /specs/002-homepage-textbook-content/spec.md
**Input**: Feature specification from `/specs/002-homepage-textbook-content/spec.md`

## Summary

This plan outlines the steps to design a minimal homepage UI and create or extend textbook documentation covering Physical AI & Humanoid Robotics. The technical approach will leverage Docusaurus for both UI design and content management, focusing on extending existing textbook content and creating new chapters while adhering to strict UI/UX guidelines (simplicity, readability, mobile-friendliness, Docusaurus native patterns, consistent theming). A critical aspect is ensuring the frontend is 100% Vercel-ready and deployable without modification, with the Docusaurus build being fully static.

## Technical Context

**Language/Version**: TypeScript (for Docusaurus configuration and React components), JavaScript (for Docusaurus scripts if needed).
**Primary Dependencies**: Docusaurus, React, related Docusaurus packages.
**Storage**: Filesystem (for Docusaurus content and static assets).
**Testing**: Manual verification by running `npm start` in the `/frontend` directory for visual inspection of UI/UX, content validation (length, links, images, tables), and deployment testing on Vercel.
**Target Platform**: Web (modern desktop and mobile browsers).
**Project Type**: Web application (Docusaurus frontend within the `/frontend` directory).
**Performance Goals**: Homepage loads and is fully navigable on desktop and mobile devices within 3 seconds on a standard broadband connection.
**Constraints**:
- Docusaurus project and configuration assumed to exist from Spec 1.
- Book content may already exist and MUST NOT be overwritten; only extended or added.
- Frontend MUST be 100% Vercel-ready and deployable on Vercel without modification.
- Docusaurus build MUST be fully static, with no server-side dependencies.
- UI must be simple, readable, mobile-friendly, documentation-first (Docusaurus native patterns).
- Documentation pages MUST use full viewport height for BOTH sidebars (left docs sidebar and right table-of-contents sidebar).
- Documentation pages UI MUST visually align with the Navbar.
- Light, Dark, and System theme modes MUST apply consistently across Navbar, Documentation pages, Sidebars.
- No theme or style mismatch is allowed between pages and Navbar.
- Use only Docusaurus-native theming and layout patterns.
- Avoid custom CSS that breaks responsiveness or theming.
**Scale/Scope**: Four modules, each chapter minimum 150 lines, including external reference links, images or diagrams, and tables where appropriate. Writing style should be structured, educational, and suitable for advanced learners.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **SINGLE-BRANCH DEVELOPMENT ONLY**: The project adheres to this rule; all work will be performed directly on the `main` branch.
- **Frontend Technologies**: Utilizes Docusaurus, React, and TypeScript, consistent with the specified frontend technologies.
- **Deployment Platforms**: Explicitly targets Vercel for frontend deployment, aligning with the constitution.

## Project Structure

### Documentation (this feature)

```text
specs/002-homepage-textbook-content/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (N/A for this spec)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── docusaurus.config.ts # Main Docusaurus configuration
├── sidebars.ts          # Sidebar configuration
├── src/
│   ├── components/      # React components (e.g., custom Docusaurus components, HomepageFeatures)
│   ├── pages/           # Docusaurus pages (e.g., index.tsx for homepage)
│   └── css/             # Custom CSS
└── docs/                # Textbook content (modules and chapters)
    ├── Module-1-ROS 2/
    ├── Module-2-Gazebo-Unity/
    ├── Module-3-ISAAC/
    └── Module-4-VLA/
```

**Structure Decision**: The "Web application" structure is adopted, focusing on the `/frontend` directory for Docusaurus. The structure for `docs/` is detailed to align with the 4-module requirement for textbook content. Specific Docusaurus configuration files (`docusaurus.config.ts`, `sidebars.ts`) and common UI development directories (`src/components/`, `src/pages/`, `src/css/`) are highlighted.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |