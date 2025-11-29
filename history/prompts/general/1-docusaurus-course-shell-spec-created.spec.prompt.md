---
id: 1
title: Docusaurus Course Shell Spec Created
stage: spec
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: docusaurus-course-shell
branch: 1-docusaurus-course-shell
user: null
command: /sp.specify Goal: Create the initial Docusaurus-based textbook shell for the "Physical AI & Humanoid Robotics" course. This shell must follow our constitution in .specify/memory/constitution.md and support future AI-native features: RAG chatbot, authentication, personalization, and Urdu translation. Context: - This repo already has Spec-Kit Plus and a filled constitution. - The course has 4 main modules + a capstone: - Module 1: ROS 2 (Robotic Nervous System) - Module 2: Gazebo & Unity (Digital Twin) - Module 3: NVIDIA Isaac (AI-Robot Brain) - Module 4: Vision-Language-Action (VLA) - Capstone: Autonomous Humanoid - Later features (rag-backend, auth, personalization, translation) will build on this Docusaurus shell. In Scope: - Choose appropriate Docusaurus 2 template and initialize the site. - Define high-level information architecture: - Landing page / overview of Physical AI & Humanoid Robotics. - Module sections (1–4) and a Capstone section. - Additional top-level pages: Preface, Hardware Requirements, Lab Architecture, About. - Decide folder + file structure under `docs/` (and any `src/pages/` if needed), including slugs and titles. - Configure basic navbar and sidebar to reflect course structure. - Configure GitHub Pages deployment (baseUrl, organizationName, projectName, etc.). - Add placeholder markdown files for each module, week breakdown, and capstone, with headings and TODO markers for future content. Out of Scope (for THIS feature): - RAG chatbot backend or frontend UI. - Authentication, personalization, or translation logic. - Detailed lesson content; only skeleton/outline. - CI pipelines beyond minimal Docusaurus deployment config. Constraints: - Must follow the constitution (decision frameworks, minimal sufficient content, anti-convergence). - Use latest stable Docusaurus 2. - Information architecture must align with the weekly breakdown and module descriptions from the Physical AI course. - Structure must be AI-friendly (clear headings, stable slugs, good IDs for embedding and retrieval). Success Criteria: - A new Docusaurus project builds and runs locally. - Navbar and sidebar expose: Home, Preface, Modules 1–4, Capstone, Hardware/ Lab pages. - `docs/` directory contains placeholder markdown files for: - Each module (overview + weekly breakdown), - Capstone project, - Hardware requirements and lab architecture. - GitHub Pages config is ready so we can deploy after implementation.
labels: ["docusaurus", "spec", "course-shell", "ai-robotics"]
links:
  spec: specs/1-docusaurus-course-shell/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-docusaurus-course-shell/spec.md
  - specs/1-docusaurus-course-shell/checklists/requirements.md
tests:

---

## Prompt

Goal:
Create the initial Docusaurus-based textbook shell for the "Physical AI & Humanoid Robotics" course. This shell must follow our constitution in .specify/memory/constitution.md and support future AI-native features: RAG chatbot, authentication, personalization, and Urdu translation.

Context:
- This repo already has Spec-Kit Plus and a filled constitution.
- The course has 4 main modules + a capstone:
  - Module 1: ROS 2 (Robotic Nervous System)
  - Module 2: Gazebo & Unity (Digital Twin)
  - Module 3: NVIDIA Isaac (AI-Robot Brain)
  - Module 4: Vision-Language-Action (VLA)
  - Capstone: Autonomous Humanoid
- Later features (rag-backend, auth, personalization, translation) will build on this Docusaurus shell.

In Scope:
- Choose appropriate Docusaurus 2 template and initialize the site.
- Define high-level information architecture:
  - Landing page / overview of Physical AI & Humanoid Robotics.
  - Module sections (1–4) and a Capstone section.
  - Additional top-level pages: Preface, Hardware Requirements, Lab Architecture, About.
- Decide folder + file structure under `docs/` (and any `src/pages/` if needed), including slugs and titles.
- Configure basic navbar and sidebar to reflect course structure.
- Configure GitHub Pages deployment (baseUrl, organizationName, projectName, etc.).
- Add placeholder markdown files for each module, week breakdown, and capstone, with headings and TODO markers for future content.

Out of Scope (for THIS feature):
- RAG chatbot backend or frontend UI.
- Authentication, personalization, or translation logic.
- Detailed lesson content; only skeleton/outline.
- CI pipelines beyond minimal Docusaurus deployment config.

Constraints:
- Must follow the constitution (decision frameworks, minimal sufficient content, anti-convergence).
- Use latest stable Docusaurus 2.
- Information architecture must align with the weekly breakdown and module descriptions from the Physical AI course.
- Structure must be AI-friendly (clear headings, stable slugs, good IDs for embedding and retrieval).

Success Criteria:
- A new Docusaurus project builds and runs locally.
- Navbar and sidebar expose: Home, Preface, Modules 1–4, Capstone, Hardware/ Lab pages.
- `docs/` directory contains placeholder markdown files for:
  - Each module (overview + weekly breakdown),
  - Capstone project,
  - Hardware requirements and lab architecture.
- GitHub Pages config is ready so we can deploy after implementation.

## Response snapshot

The feature specification for `docusaurus-course-shell` has been successfully created and validated.
Branch Name: `1-docusaurus-course-shell`
Specification File: `specs/1-docusaurus-course-shell/spec.md`
Checklist File: `specs/1-docusaurus-course-shell/checklists/requirements.md`
All specification quality checks passed, and there are no outstanding clarifications needed. This feature is now ready for the next phase, `/sp.clarify` or `/sp.plan`.

## Outcome

- ✅ Impact: Spec created and validated, ready for next steps.
- 🧪 Tests: N/A
- 📁 Files: `specs/1-docusaurus-course-shell/spec.md` and `specs/1-docusaurus-course-shell/checklists/requirements.md` created.
- 🔁 Next prompts: /sp.clarify or /sp.plan
- 🧠 Reflection: Manual PHR creation due to `create-new-feature.sh` script failure when parsing complex multi-line arguments in PowerShell.

## Evaluation notes (flywheel)

- Failure modes observed: `create-new-feature.sh` script failed due to argument parsing for multi-line string with special characters like backticks and ampersands in PowerShell.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Investigate `create-new-feature.sh` argument parsing for complex strings in PowerShell and implement robust escaping or alternative argument passing.
