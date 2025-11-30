# Repository Setup Guide

## The Problem
You're getting a 404 error when trying to access the GitHub Pages settings. This usually means:
1. The repository doesn't exist on GitHub yet
2. The repository name is different
3. You don't have access to the repository

## Solution: Create or Verify Repository

### Step 1: Check Your Current Git Remote

Run this command to see your current repository:
```bash
git remote -v
```

### Step 2: Create Repository on GitHub (If It Doesn't Exist)

**Option A: Using GitHub Website**

1. Go to: https://github.com/new
2. Repository name: `Physical-AI-and-Humanoid-Robotics`
3. Description: "Physical AI & Humanoid Robotics Course Textbook"
4. Visibility: **Public** (required for free GitHub Pages)
5. **DO NOT** initialize with README, .gitignore, or license (you already have files)
6. Click **Create repository**

**Option B: Using GitHub CLI (If Installed)**

```bash
gh repo create Physical-AI-and-Humanoid-Robotics --public --source=. --remote=origin --push
```

### Step 3: Connect Your Local Repository to GitHub

If the repository doesn't exist or remote is wrong:

```bash
# Remove old remote (if exists)
git remote remove origin

# Add correct remote
git remote add origin https://github.com/Alisha-Kayani/Physical-AI-and-Humanoid-Robotics.git

# Push your code
git push -u origin 001-docusaurus-config
```

### Step 4: Push All Your Code

Make sure all files are committed and pushed:

```bash
# Check status
git status

# Add all files
git add .

# Commit
git commit -m "Initial Docusaurus setup with content"

# Push to GitHub
git push origin 001-docusaurus-config
```

### Step 5: Enable GitHub Pages

Once the repository exists and code is pushed:

1. Go to: `https://github.com/Alisha-Kayani/Physical-AI-and-Humanoid-Robotics`
2. Click **Settings** (top menu)
3. Click **Pages** (left sidebar)
4. Under **Source**, select:
   - **Source**: `Deploy from a branch`
   - **Branch**: `gh-pages` → `/ (root)`
5. Click **Save**

**OR** if you want to use GitHub Actions:

1. Go to: `https://github.com/Alisha-Kayani/Physical-AI-and-Humanoid-Robotics/settings/pages`
2. Under **Source**, select: **GitHub Actions**
3. Click **Save**

## Alternative: Check Repository Name

If the repository exists but with a different name:

1. Go to: https://github.com/Alisha-Kayani?tab=repositories
2. Find your repository (might be named differently)
3. Update `docusaurus.config.js` to match the actual repository name:

```javascript
baseUrl: '/ACTUAL-REPO-NAME/',
projectName: 'ACTUAL-REPO-NAME',
```

## Verify Repository Exists

Check if the repository is accessible:
- Go to: `https://github.com/Alisha-Kayani/Physical-AI-and-Humanoid-Robotics`
- If you see a 404, the repository doesn't exist
- If you see the repository, check the name matches exactly

## Quick Checklist

- [ ] Repository exists on GitHub
- [ ] Repository name matches: `Physical-AI-and-Humanoid-Robotics`
- [ ] Local code is pushed to GitHub
- [ ] `gh-pages` branch exists (we already deployed it)
- [ ] GitHub Pages is enabled in Settings → Pages
- [ ] Source is set to "Deploy from a branch" → `gh-pages`

## After Setup

Your site will be available at:
`https://Alisha-Kayani.github.io/Physical-AI-and-Humanoid-Robotics/`

Wait 1-2 minutes after enabling Pages for it to become live.

