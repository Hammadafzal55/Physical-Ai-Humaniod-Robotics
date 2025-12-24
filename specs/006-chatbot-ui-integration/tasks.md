# Tasks: Chatbot UI Integration

**Input**: Design documents from `/specs/006-chatbot-ui-integration/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)

## Path Conventions

- **Frontend**: `frontend/`
- **Backend**: `backend/`

---

## Phase 1: Setup (Frontend Environment)

**Purpose**: Prepare the Docusaurus environment to accept a global UI component.

- [X] T001 Use the Docusaurus CLI to "swizzle" the `Layout` component, creating a copy at `frontend/src/theme/Layout/index.tsx`.
- [X] T002 [P] Create an empty `ChatContext.tsx` in `frontend/src/context/` to manage the chat's open/closed state.
- [X] T003 Modify the new `frontend/src/theme/Layout/index.tsx` to wrap its children with the `ChatContext.Provider`.

---

## Phase 2: Foundational UI (User Story 2)

**Goal**: Display a non-functional chatbot icon and an empty chat window that can be opened and closed.

**Independent Test**: A user can navigate to any page on the site, see the chat icon, click it, and see a chat window appear with the initial greeting. The window can be closed.

### Implementation for User Story 2

- [X] T004 [P] [US2] Create the `ChatLauncher` component in `frontend/src/components/ChatLauncher/index.tsx` that renders a floating robot icon button.
- [X] T005 [P] [US2] Create a scaffold for the `ChatWindow` component in `frontend/src/components/ChatWindow/index.tsx` that displays a static greeting message.
- [X] T006 [P] [US2] Add initial CSS styles for the launcher and window to `frontend/src/css/custom.css` to position them correctly and match the site's theme.
- [ ] T007 [US2] Import and render the `ChatLauncher` and `ChatWindow` components within the swizzled `frontend/src/theme/Layout/index.tsx`. (Depends on T004, T005)
- [X] T008 [US2] Implement the state logic in `ChatContext.tsx` and connect it to the `ChatLauncher` and `ChatWindow` to toggle visibility. (Depends on T002, T007)

**Checkpoint**: At this point, the chatbot UI should be visible and interactive on all pages, but not yet connected to the backend.

---

## Phase 3: Backend Streaming Implementation

**Purpose**: Modify the backend to support streaming responses, as required by `FR-011`. This is a prerequisite for the full end-to-end integration.

- [X] T009 [P] Based on the outcome of `research.md` (R-02), modify the `GeminiRagAgent.chat()` method in `backend/agent.py` to be an `async` generator that `yield`s response chunks.
- [X] T010 [P] Update the `/chat` endpoint in `backend/api.py` to be an `async def` function that returns a `StreamingResponse`.
- [X] T011 Update the `/chat` endpoint in `backend/api.py` to call the agent's new async generator and stream its yielded chunks to the client. (Depends on T009, T010)

**Checkpoint**: The `/chat` API endpoint can now be manually tested (e.g., with `curl`) and should return a `text/event-stream` response.

---

## Phase 4: Full End-to-End Integration (User Story 1) 🎯 MVP

**Goal**: Connect the frontend chat UI to the streaming backend API to create a fully functional chatbot experience.

**Independent Test**: A user can ask a question in the chat UI and see the agent's response get progressively rendered in the chat window.

### Implementation for User Story 1

- [X] T012 [US1] Implement the `fetch` API logic within `frontend/src/components/ChatWindow/index.tsx` to call the `POST /chat` backend endpoint.
- [X] T013 [US1] Add logic to `frontend/src/components/ChatWindow/index.tsx` to read the `text/event-stream` from the `fetch` response. (Depends on T012)
- [X] T014 [US1] Update the React state in `frontend/src/components/ChatWindow/index.tsx` as new text chunks are received, causing the response to be progressively rendered in the UI. (Depends on T013)
- [X] T015 [P] [US1] Implement user message history, so the user's query is added to the message list immediately upon submission.
- [X] T016 [P] [US1] Implement graceful error handling in the UI to display a message if the API call fails.

**Checkpoint**: User Story 1 is fully functional. The chatbot is integrated end-to-end.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements to ensure a high-quality, demo-ready feature.

- [X] T017 [P] Conduct a full review of the chatbot UI on multiple mobile device sizes and fix any layout or responsiveness issues.
- [X] T018 [P] Update component styles in `frontend/src/css/custom.css` to ensure perfect visual consistency with the textbook theme.
- [X] T019 Perform a final validation of the end-to-end flow by following the steps in `specs/006-chatbot-ui-integration/quickstart.md`.

---

## Dependencies & Execution Order

### Phase Dependencies

1.  **Setup (Phase 1)**: Must be completed first.
2.  **Foundational UI (Phase 2)**: Depends on Setup.
3.  **Backend Streaming (Phase 3)**: Can be done in parallel with Phase 1 and 2.
4.  **Full Integration (Phase 4)**: Depends on both Foundational UI (Phase 2) and Backend Streaming (Phase 3).
5.  **Polish (Phase 5)**: Depends on all previous phases being complete.

### User Story Dependencies

-   **User Story 2 (P2)**: Implemented in Phase 2. Can be completed and tested before backend work begins.
-   **User Story 1 (P1)**: Implemented in Phase 4. Depends on the UI from US2 and the backend streaming logic.

### Parallel Opportunities

-   **Phase 2 and 3 can be done in parallel**: A frontend developer can work on the UI components (Phase 2) while a backend developer works on the streaming API (Phase 3).
-   Within Phase 2, tasks T004, T005, and T006 are parallelizable.
-   Within Phase 3, tasks T009 and T010 are parallelizable.
-   Within Phase 4, tasks T015 and T016 are parallelizable.

---

## Implementation Strategy

### MVP First (User Story 2 + Non-Streaming US1)

1.  Complete Phase 1 (Setup).
2.  Complete Phase 2 (Foundational UI).
3.  **STOP and VALIDATE**: The UI is visible and interactive.
4.  Implement a *non-streaming* version of Phase 4 (Full Integration) if the backend streaming (Phase 3) is blocked or delayed. This delivers the core value, albeit with a slower-feeling UX.

### Incremental Delivery (Recommended)

1.  Complete Phases 1, 2, and 3.
2.  **Checkpoint**: UI is ready, and backend API is streaming.
3.  Complete Phase 4 to connect them.
4.  **STOP and VALIDATE**: Test the full end-to-end streaming experience.
5.  Complete Phase 5 for final polish.
