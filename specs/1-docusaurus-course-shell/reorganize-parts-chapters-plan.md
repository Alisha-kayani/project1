# Plan: Reorganize Structure to Parts and Chapters

## Overview

Reorganize the Docusaurus sidebar structure from "Modules" and "Weeks" to "Parts" and "Chapters" to match the reference "AI Native Development Book" interface pattern. This creates a more book-like structure while maintaining all existing content.

## Current Structure

**Current Sidebar**:
- Preface
- Module 1: ROS 2 (Robotic Nervous System)
  - Overview
  - Week 1: ROS 2 Fundamentals
  - Week 2: ROS 2 Services and Advanced Patterns
- Module 2: Gazebo & Unity (Digital Twin)
  - Overview
  - Week 1: Gazebo Simulation Basics
  - Week 2: Unity Integration
- Module 3: NVIDIA Isaac (AI-Robot Brain)
  - Overview
  - Week 1: NVIDIA Isaac Platform Introduction
  - Week 2: AI-Powered Perception
- Module 4: Vision-Language-Action (VLA)
  - Overview
  - Week 1: VLA Fundamentals
  - Week 2: LLM-Controlled Robotics
- Capstone: Autonomous Humanoid
  - Overview
- Hardware Requirements
- Lab Architecture
- About

## New Structure (Parts and Chapters)

**New Sidebar**:
- Preface: Welcome to Physical AI & Humanoid Robotics
- Part 1: ROS 2 (Robotic Nervous System)
  - Chapter 1: ROS 2 Overview
  - Chapter 2: ROS 2 Fundamentals
  - Chapter 3: ROS 2 Services and Advanced Patterns
- Part 2: Gazebo & Unity (Digital Twin)
  - Chapter 1: Digital Twin Overview
  - Chapter 2: Gazebo Simulation Basics
  - Chapter 3: Unity Integration and Sim-to-Real
- Part 3: NVIDIA Isaac (AI-Robot Brain)
  - Chapter 1: AI-Robot Brain Overview
  - Chapter 2: Isaac Sim and Photorealistic Simulation
  - Chapter 3: Isaac ROS and AI-Powered Perception
- Part 4: Vision-Language-Action (VLA)
  - Chapter 1: VLA Overview
  - Chapter 2: VLA Fundamentals
  - Chapter 3: LLM-Controlled Robotics
- Capstone: Autonomous Humanoid
  - Overview
- Hardware Requirements
- Lab Architecture
- About

## Changes Required

### 1. Update Sidebar Labels (sidebars.js)

**Changes**:
- Change "Module 1: ROS 2" → "Part 1: ROS 2 (Robotic Nervous System)"
- Change "Module 2: Gazebo & Unity" → "Part 2: Gazebo & Unity (Digital Twin)"
- Change "Module 3: NVIDIA Isaac" → "Part 3: NVIDIA Isaac (AI-Robot Brain)"
- Change "Module 4: VLA" → "Part 4: Vision-Language-Action (VLA)"
- Change "Week 1" items → "Chapter 2" (Overview becomes Chapter 1)
- Change "Week 2" items → "Chapter 3"
- Keep "Overview" items but label as "Chapter 1" in sidebar

### 2. Update Page Titles (Frontmatter)

**Files to Update**:
- All module overview pages: Update titles to include "Chapter 1" or keep as "Overview"
- All week-1 pages: Update titles from "Week 1" to "Chapter 2"
- All week-2 pages: Update titles from "Week 2" to "Chapter 3"

**Decision**: Keep page titles descriptive but update sidebar labels to show "Chapter X" format.

### 3. Update Page Content Headings

**Optional**: Update H1 headings in markdown files to reflect "Chapter" terminology if desired, or keep current headings and only change sidebar labels.

**Recommendation**: Update sidebar labels only, keep page content headings as-is for clarity (e.g., "Week 1: ROS 2 Fundamentals" is clearer than "Chapter 2: ROS 2 Fundamentals" in the actual page).

## Implementation Steps

1. **Update sidebars.js**:
   - Change category labels from "Module X" to "Part X"
   - Change item labels to "Chapter 1", "Chapter 2", "Chapter 3" format
   - Keep file references the same (no file moves needed)

2. **Update Page Titles (Optional)**:
   - Update frontmatter `title` fields to include "Chapter" terminology
   - Or keep titles as-is and only change sidebar display

3. **Update Preface Title**:
   - Change to "Preface: Welcome to Physical AI & Humanoid Robotics" (matches reference pattern)

4. **Verify Structure**:
   - Test sidebar navigation
   - Verify all pages accessible
   - Check right sidebar TOC still works
   - Build and test

## Files to Modify

1. `sidebars.js` - Update all category and item labels
2. `docs/preface.md` - Update title to match reference pattern (optional)
3. Page frontmatter titles (optional - can keep current titles)

## Success Criteria

- Sidebar shows "Part 1", "Part 2", etc. instead of "Module 1", "Module 2"
- Sidebar shows "Chapter 1", "Chapter 2", "Chapter 3" for each part
- All pages remain accessible
- Navigation structure matches reference interface pattern
- Build succeeds without errors
- Right sidebar TOC continues to work

## Design Decisions

**Terminology**:
- Parts = Main course sections (formerly Modules)
- Chapters = Individual lessons within parts (formerly Overview + Weeks)
- Preface = Introduction section (keep as top-level)

**Labeling Strategy**:
- Sidebar labels: Use "Part X" and "Chapter X" format
- Page titles: Can keep descriptive names (e.g., "Week 1: ROS 2 Fundamentals") or update to "Chapter 2: ROS 2 Fundamentals"
- **Recommendation**: Update sidebar labels, keep page titles descriptive for clarity

**Structure Mapping**:
- Module 1 → Part 1
- Module 2 → Part 2
- Module 3 → Part 3
- Module 4 → Part 4
- Overview → Chapter 1
- Week 1 → Chapter 2
- Week 2 → Chapter 3
- Capstone → Keep as separate section (not a Part)

