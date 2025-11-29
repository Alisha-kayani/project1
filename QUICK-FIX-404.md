# Quick Fix for GitHub Pages 404 Error

## The Problem
You're seeing a 404 error because GitHub Pages hasn't been deployed yet.

## Quick Solution (3 Steps)

### Step 1: Push Your Code to GitHub

Make sure all your files (including the workflow) are pushed:

```bash
git add .
git commit -m "Add deployment workflow and fix config"
git push origin 001-docusaurus-config
```

### Step 2: Enable GitHub Pages

1. Go to: `https://github.com/Alisha-Kayani/Physical-AI-and-Humanoid-Robotics/settings/pages`
2. Under **Source**, select: **GitHub Actions** (NOT "Deploy from a branch")
3. Click **Save**

### Step 3: Trigger the Deployment

The workflow will run automatically when you push. To trigger it manually:

1. Go to: `https://github.com/Alisha-Kayani/Physical-AI-and-Humanoid-Robotics/actions`
2. Click on "Deploy to GitHub Pages" workflow
3. Click "Run workflow" → Select your branch → "Run workflow"

**OR** make a small commit to trigger it:

```bash
echo "" >> README.md
git add README.md
git commit -m "Trigger deployment"
git push origin 001-docusaurus-config
```

## Wait and Check

1. Wait 2-5 minutes for the workflow to complete
2. Check the Actions tab to see if it succeeded
3. Once complete, your site will be live at:
   `https://Alisha-Kayani.github.io/Physical-AI-and-Humanoid-Robotics/`

## If It Still Doesn't Work

### Check Workflow Permissions

1. Go to: `https://github.com/Alisha-Kayani/Physical-AI-and-Humanoid-Robotics/settings/actions`
2. Under "Workflow permissions", select: **Read and write permissions**
3. Check: **Allow GitHub Actions to create and approve pull requests**
4. Save

### Verify Configuration

Make sure in `docusaurus.config.js`:
- `url: 'https://Alisha-Kayani.github.io'`
- `baseUrl: '/Physical-AI-and-Humanoid-Robotics/'`
- `organizationName: 'Alisha-Kayani'`
- `projectName: 'Physical-AI-and-Humanoid-Robotics'`

All should match your repository name exactly!

## Alternative: Manual Deploy

If GitHub Actions doesn't work, use this:

```bash
npm install --save-dev gh-pages
npm run deploy
```

Then in GitHub Settings → Pages:
- Source: **Deploy from a branch**
- Branch: **gh-pages** → **/ (root)**
- Save

