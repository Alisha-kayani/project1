# Implementation Plan: Docusaurus Course Shell Setup

**Branch**: `1-docusaurus-course-shell` | **Date**: 2025-11-29 | **Spec**: [specs/1-docusaurus-course-shell/spec.md](./spec.md)

## Summary

This plan outlines the setup and configuration of a Docusaurus 2-based textbook shell for the "Physical AI & Humanoid Robotics" course. The implementation will create a functional documentation site with proper navigation structure, placeholder content files, and GitHub Pages deployment configuration. The structure is designed to be AI-friendly (clear headings, stable slugs) to support future RAG chatbot, authentication, personalization, and Urdu translation features.

**Primary Requirement**: Initialize Docusaurus 2 project with course structure (4 modules + capstone), configure navigation, create placeholder markdown files, and prepare for GitHub Pages deployment.

**Technical Approach**: Use Docusaurus 2 classic template, configure via `docusaurus.config.js` and `sidebars.js`, organize content in `docs/` directory with clear hierarchical structure.

## Technical Context

**Language/Version**: Node.js 18+ (LTS), Docusaurus 2.x (latest stable)  
**Primary Dependencies**: 
- `@docusaurus/core` (latest stable 2.x)
- `@docusaurus/preset-classic` (latest stable 2.x)
- React 17/18 (bundled with Docusaurus)

**Storage**: File-based (Markdown files in `docs/` directory)  
**Testing**: Manual verification (build/test commands, browser testing)  
**Target Platform**: Web (static site, deployable to GitHub Pages)  
**Project Type**: Documentation site (single project)  
**Performance Goals**: Fast initial load, responsive navigation, SEO-friendly URLs  
**Constraints**: 
- Must use Docusaurus 2 (not v3)
- Structure must be AI-friendly (stable slugs, clear IDs)
- Must follow constitution principles (minimal sufficient content, anti-convergence)
- Must support future AI-native features (RAG, auth, personalization, translation)

**Scale/Scope**: 
- 4 main modules + 1 capstone
- ~20-30 placeholder markdown files
- Single documentation site

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **Minimal Sufficient Content**: Only placeholder files with headings and TODO markers—no detailed content  
✅ **AI-Friendly Structure**: Clear headings, stable slugs, good IDs for embedding/retrieval  
✅ **Decision Frameworks**: Information architecture decisions documented with rationale  
✅ **Anti-Convergence**: Custom course structure (not generic Docusaurus template)  
✅ **Reversible Decisions**: Configuration choices are easily changeable

## Project Structure

### Documentation (this feature)

```text
specs/1-docusaurus-course-shell/
├── plan.md              # This file
├── spec.md              # Feature specification
└── checklists/
    └── requirements.md  # Quality checklist
```

### Source Code (repository root)

```text
project1/
├── docs/                          # Course content (Markdown files)
│   ├── preface.md                 # Course preface
│   ├── hardware-requirements.md   # Hardware requirements page
│   ├── lab-architecture.md        # Lab architecture page
│   ├── about.md                   # About page
│   ├── module-1-ros2/            # Module 1: ROS 2
│   │   ├── overview.md           # Module overview
│   │   ├── week-1.md             # Week 1 content
│   │   ├── week-2.md             # Week 2 content
│   │   └── ...                   # Additional weeks
│   ├── module-2-gazebo-unity/    # Module 2: Gazebo & Unity
│   │   ├── overview.md
│   │   └── ...                   # Weekly breakdowns
│   ├── module-3-nvidia-isaac/    # Module 3: NVIDIA Isaac
│   │   ├── overview.md
│   │   └── ...                   # Weekly breakdowns
│   ├── module-4-vla/             # Module 4: Vision-Language-Action
│   │   ├── overview.md
│   │   └── ...                   # Weekly breakdowns
│   └── capstone/                 # Capstone project
│       └── overview.md
├── src/
│   ├── pages/
│   │   └── index.js              # Custom landing page (optional)
│   └── css/
│       └── custom.css            # Custom styles (if needed)
├── static/                        # Static assets (images, etc.)
├── docusaurus.config.js          # Main Docusaurus configuration
├── sidebars.js                    # Sidebar navigation structure
├── package.json                   # Node.js dependencies
├── package-lock.json              # Dependency lock file
└── .gitignore                     # Git ignore rules
```

**Structure Decision**: 
- Use `docs/` directory for all course content (standard Docusaurus pattern)
- Organize modules as subdirectories with `overview.md` + weekly files
- Use kebab-case for file names (stable, URL-friendly slugs)
- Separate top-level pages (preface, hardware, lab, about) at docs root
- Custom landing page in `src/pages/index.js` (optional, can use default)

## Phase 0: Research & Prerequisites

### 0.1 Docusaurus 2 Installation Requirements

**Research Tasks**:
- [ ] Verify Node.js version compatibility (Docusaurus 2 requires Node.js 16.14+ or 18+)
- [ ] Check latest stable Docusaurus 2.x version
- [ ] Review Docusaurus 2 documentation for classic template
- [ ] Review GitHub Pages deployment guide for Docusaurus

**Key Findings**:
- Docusaurus 2 classic template includes docs, blog, and pages plugins
- GitHub Pages deployment requires `baseUrl` configuration
- Sidebar structure defined in `sidebars.js` using autogenerated or manual configuration

### 0.2 Information Architecture Design

**Decision Points**:
1. **Module Organization**: Subdirectories vs. flat structure
   - **Decision**: Subdirectories (`module-1-ros2/`, etc.) for better organization
   - **Rationale**: Easier to manage, clearer URLs, supports future expansion

2. **Weekly Content Structure**: Separate files vs. single file per module
   - **Decision**: Separate files per week (`week-1.md`, `week-2.md`, etc.)
   - **Rationale**: Better for navigation, easier content management, AI-friendly

3. **Landing Page**: Default vs. Custom
   - **Decision**: Start with default, can customize later
   - **Rationale**: Minimal sufficient content per constitution

4. **Slug Strategy**: Auto-generated vs. Explicit
   - **Decision**: Use explicit `id` frontmatter for stable slugs
   - **Rationale**: AI-friendly, stable URLs for embedding/retrieval

### 0.3 GitHub Pages Configuration

**Required Settings**:
- `url`: Full GitHub Pages URL (e.g., `https://username.github.io`)
- `baseUrl`: Repository name with leading slash (e.g., `/project1/`)
- `organizationName`: GitHub username or organization
- `projectName`: Repository name

**Note**: These values need to be determined based on actual GitHub repository details.

## Phase 1: Design & Architecture

### 1.1 File Structure Design

**Module Structure Pattern**:
```
docs/module-{N}-{slug}/
├── overview.md          # Module introduction
├── week-1.md           # Week 1 content
├── week-2.md           # Week 2 content
└── ...                 # Additional weeks
```

**File Naming Convention**:
- Use kebab-case: `module-1-ros2`, `hardware-requirements`
- Stable, URL-friendly slugs
- Consistent pattern across all files

**Frontmatter Template**:
```yaml
---
id: stable-slug-identifier
title: Human-Readable Title
sidebar_position: 1
---
```

### 1.2 Navigation Structure

**Navbar Items** (top navigation):
1. Home (default)
2. Preface
3. Module 1: ROS 2
4. Module 2: Gazebo & Unity
5. Module 3: NVIDIA Isaac
6. Module 4: VLA
7. Capstone
8. Hardware Requirements
9. Lab Architecture
10. About

**Sidebar Structure** (hierarchical):
```
- Preface
- Module 1: ROS 2 (Robotic Nervous System)
  - Overview
  - Week 1: [Topic]
  - Week 2: [Topic]
  - ...
- Module 2: Gazebo & Unity (Digital Twin)
  - Overview
  - Week 1: [Topic]
  - ...
- Module 3: NVIDIA Isaac (AI-Robot Brain)
  - Overview
  - Week 1: [Topic]
  - ...
- Module 4: Vision-Language-Action (VLA)
  - Overview
  - Week 1: [Topic]
  - ...
- Capstone: Autonomous Humanoid
  - Overview
- Hardware Requirements
- Lab Architecture
- About
```

### 1.3 Configuration Design

**docusaurus.config.js Key Settings**:
- `title`: "Physical AI & Humanoid Robotics"
- `tagline`: [To be determined - course tagline]
- `url`: [GitHub Pages URL]
- `baseUrl`: [Repository path]
- `organizationName`: [GitHub org/username]
- `projectName`: [Repository name]
- `deploymentBranch`: `gh-pages` (for GitHub Pages)
- `trailingSlash`: `false` (clean URLs)

**sidebars.js Structure**:
- Manual sidebar configuration (not autogenerated)
- Hierarchical structure matching course modules
- Consistent ordering and grouping

## Phase 2: Implementation Tasks

### Task 1: Initialize Docusaurus Project

**Steps**:
1. Install Docusaurus CLI globally (if not already installed): `npm install -g @docusaurus/init`
2. Initialize project: `npx create-docusaurus@latest . classic --typescript false`
   - **Note**: Use `.` to initialize in current directory
   - Choose "classic" template
   - Skip TypeScript (use JavaScript)
3. Verify installation: `npm start` should launch dev server
4. Test build: `npm run build` should generate `build/` directory

**Acceptance Criteria**:
- [ ] `package.json` contains Docusaurus 2.x dependencies
- [ ] `docusaurus.config.js` exists and is valid
- [ ] `docs/` directory exists
- [ ] Local dev server runs without errors
- [ ] Build completes successfully

### Task 2: Configure docusaurus.config.js

**Steps**:
1. Update `title` to "Physical AI & Humanoid Robotics"
2. Set appropriate `tagline`
3. Configure GitHub Pages settings:
   - `url`: [To be filled with actual GitHub Pages URL]
   - `baseUrl`: [To be filled with repository path]
   - `organizationName`: [To be filled]
   - `projectName`: [To be filled]
4. Set `deploymentBranch` to `gh-pages`
5. Configure `themeConfig.navbar` with course navigation items
6. Set `trailingSlash: false` for clean URLs

**Acceptance Criteria**:
- [ ] Site title displays correctly in browser
- [ ] GitHub Pages configuration is present (even if values are placeholders)
- [ ] Navbar contains all required links
- [ ] Build generates correct base URL paths

### Task 3: Configure sidebars.js

**Steps**:
1. Replace autogenerated sidebar with manual configuration
2. Define structure matching Phase 1.2 design
3. Map each sidebar item to corresponding markdown file
4. Set appropriate `sidebar_position` in frontmatter for ordering

**Acceptance Criteria**:
- [ ] Sidebar displays hierarchical module structure
- [ ] All modules, weeks, and top-level pages are accessible
- [ ] Navigation is intuitive and matches course structure

### Task 4: Create Directory Structure

**Steps**:
1. Create module subdirectories:
   - `docs/module-1-ros2/`
   - `docs/module-2-gazebo-unity/`
   - `docs/module-3-nvidia-isaac/`
   - `docs/module-4-vla/`
   - `docs/capstone/`
2. Verify directory structure matches Phase 1.1 design

**Acceptance Criteria**:
- [ ] All module directories exist
- [ ] Directory structure is consistent and organized

### Task 5: Create Placeholder Markdown Files

**Steps**:
1. Create top-level pages:
   - `docs/preface.md`
   - `docs/hardware-requirements.md`
   - `docs/lab-architecture.md`
   - `docs/about.md`
2. For each module, create:
   - `docs/module-{N}-{slug}/overview.md`
   - `docs/module-{N}-{slug}/week-1.md`
   - `docs/module-{N}-{slug}/week-2.md`
   - [Additional weeks as needed]
3. Create capstone file:
   - `docs/capstone/overview.md`
4. Each file should include:
   - Proper frontmatter with `id`, `title`, `sidebar_position`
   - Main heading matching the title
   - Section headings for content structure
   - `<!-- TODO: Add content -->` markers

**Placeholder Content Template**:
```markdown
---
id: stable-slug
title: Page Title
sidebar_position: 1
---

# Page Title

## Introduction

<!-- TODO: Add introduction content -->

## Main Content

<!-- TODO: Add main content -->

## Summary

<!-- TODO: Add summary -->
```

**Acceptance Criteria**:
- [ ] All required placeholder files exist
- [ ] Each file has proper frontmatter
- [ ] Files contain headings and TODO markers
- [ ] All files are accessible via navigation
- [ ] URLs are clean and match slug pattern

### Task 6: Verify Navigation

**Steps**:
1. Start dev server: `npm start`
2. Test all navbar links
3. Test all sidebar links
4. Verify URL structure
5. Check mobile responsiveness

**Acceptance Criteria**:
- [ ] All navbar links work correctly
- [ ] All sidebar links work correctly
- [ ] URLs are clean and readable
- [ ] Navigation is responsive on mobile devices

### Task 7: Test Build and Deployment Readiness

**Steps**:
1. Run production build: `npm run build`
2. Verify `build/` directory is generated
3. Test build output: `npm run serve`
4. Verify GitHub Pages configuration in `docusaurus.config.js`
5. Document deployment steps (for future CI/CD)

**Acceptance Criteria**:
- [ ] Build completes without errors
- [ ] Build output is valid and serves correctly
- [ ] GitHub Pages configuration is ready (values may be placeholders)
- [ ] Deployment documentation exists

## Phase 3: Validation & Testing

### 3.1 Manual Testing Checklist

- [ ] Local dev server starts without errors
- [ ] All pages load correctly
- [ ] Navigation (navbar and sidebar) works
- [ ] URLs are clean and match expected slugs
- [ ] Build completes successfully
- [ ] Build output serves correctly
- [ ] Mobile navigation is functional
- [ ] All placeholder files are accessible

### 3.2 Success Criteria Verification

**SC-001**: A new Docusaurus project successfully builds and runs locally without errors
- [ ] `npm start` runs successfully
- [ ] `npm run build` completes without errors
- [ ] No console errors in browser

**SC-002**: All required navigation links are present and functional
- [ ] Navbar contains: Home, Preface, Modules 1-4, Capstone, Hardware, Lab, About
- [ ] Sidebar reflects hierarchical structure
- [ ] All links navigate to correct pages

**SC-003**: `docs/` directory contains all specified placeholder files
- [ ] All module overview files exist
- [ ] Weekly breakdown files exist for each module
- [ ] Capstone file exists
- [ ] Top-level pages (Preface, Hardware, Lab, About) exist
- [ ] All files have proper frontmatter and TODO markers

**SC-004**: `docusaurus.config.js` is correctly configured for GitHub Pages
- [ ] `url`, `baseUrl`, `organizationName`, `projectName` are configured
- [ ] `deploymentBranch` is set to `gh-pages`
- [ ] Build generates correct asset paths

## Risk Analysis and Mitigation

### Top 3 Risks

1. **Risk**: GitHub Pages configuration values unknown at implementation time
   - **Blast Radius**: Deployment will fail if incorrect
   - **Mitigation**: Use placeholder values with clear comments, document where to update
   - **Kill Switch**: Configuration is easily editable in `docusaurus.config.js`

2. **Risk**: Module structure doesn't match actual course content needs
   - **Blast Radius**: Requires restructuring later
   - **Mitigation**: Design flexible structure, use consistent patterns, document rationale
   - **Kill Switch**: File structure is easily reorganizable

3. **Risk**: Docusaurus version compatibility issues
   - **Blast Radius**: Build failures, feature incompatibilities
   - **Mitigation**: Use latest stable 2.x, test build early, lock dependencies
   - **Kill Switch**: Can downgrade/upgrade Docusaurus version if needed

## Dependencies and Assumptions

### External Dependencies
- Node.js 18+ (LTS) installed on development machine
- npm or yarn package manager
- Git repository (for GitHub Pages deployment)
- GitHub account (for deployment)

### Assumptions
- GitHub repository will be created/configured for GitHub Pages
- Course content structure (4 modules + capstone) is final
- Weekly breakdown per module will be determined during content creation phase
- Custom landing page is optional (can use default)

### Out of Scope (Confirmed)
- RAG chatbot backend or frontend UI
- Authentication, personalization, or translation logic
- Detailed lesson content (only placeholders)
- CI/CD pipelines (beyond basic deployment config)
- Custom themes or extensive styling

## Next Steps After Implementation

1. **Content Creation**: Fill placeholder files with actual course content
2. **GitHub Pages Deployment**: Configure actual repository values and deploy
3. **CI/CD Setup**: Add GitHub Actions workflow for automatic deployment
4. **Future Features**: Plan RAG chatbot, authentication, personalization, Urdu translation

## Architectural Decision Record (ADR) Considerations

**Potential ADRs** (to be evaluated during implementation):
1. **Docusaurus 2 vs. 3**: Decision to use Docusaurus 2 (stability, ecosystem maturity)
2. **File Organization Strategy**: Subdirectories vs. flat structure for modules
3. **Navigation Architecture**: Manual sidebar vs. autogenerated

**ADR Suggestion Trigger**: If significant tradeoffs are discovered during implementation, suggest documenting with `/sp.adr <decision-title>`.

## Evaluation and Validation

### Definition of Done
- [ ] All implementation tasks completed
- [ ] All acceptance criteria met
- [ ] Manual testing checklist passed
- [ ] Success criteria verified
- [ ] Documentation updated (if needed)
- [ ] Code committed to feature branch

### Output Validation
- **Format**: Valid Docusaurus 2 project structure
- **Requirements**: Matches specification requirements (FR-001 through FR-008)
- **Safety**: No hardcoded secrets, no production dependencies in dev

## Notes

- This plan assumes starting from an empty or minimal repository
- GitHub Pages configuration values should be updated when repository details are known
- Weekly breakdown files can be added incrementally as course structure is finalized
- Custom landing page (`src/pages/index.js`) is optional and can be added later if needed

