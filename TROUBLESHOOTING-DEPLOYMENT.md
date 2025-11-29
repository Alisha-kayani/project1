# Troubleshooting: GitHub Pages 404 Error

## Current Issue
You're seeing a 404 error at `alisha-kayani.github.io/Physical-AI-and-Humanoid-Robotics/`

This means GitHub Pages hasn't been set up or deployed yet. Follow these steps:

## Step-by-Step Fix

### Step 1: Check Your Current Branch

Your current branch is `001-docusaurus-config`. The GitHub Actions workflow is set to deploy from `main` branch.

**Option A: Merge to main branch (Recommended)**
```bash
git checkout main
git merge 001-docusaurus-config
git push origin main
```

**Option B: Update workflow to deploy from your current branch**
- Edit `.github/workflows/deploy.yml` and change `main` to `001-docusaurus-config`

### Step 2: Enable GitHub Pages in Repository Settings

1. Go to your GitHub repository: `https://github.com/Alisha-Kayani/Physical-AI-and-Humanoid-Robotics`
2. Click **Settings** (top menu)
3. Scroll down to **Pages** (left sidebar)
4. Under **Source**, select:
   - **Source**: `GitHub Actions` (NOT "Deploy from a branch")
   - This will use the workflow we created
5. Click **Save**

### Step 3: Verify GitHub Actions Workflow

1. Go to **Actions** tab in your repository
2. You should see "Deploy to GitHub Pages" workflow
3. If it shows as failed, click on it to see the error
4. If it hasn't run yet, push a commit to trigger it

### Step 4: Trigger Deployment

If the workflow hasn't run automatically:

```bash
# Make a small change to trigger the workflow
git checkout main  # or your branch
echo "" >> README.md
git add .
git commit -m "Trigger GitHub Pages deployment"
git push origin main
```

### Step 5: Wait for Deployment

- GitHub Actions takes 2-5 minutes to build and deploy
- Check the **Actions** tab to see progress
- Once complete, your site will be live

## Alternative: Manual Deployment (If GitHub Actions Fails)

If GitHub Actions doesn't work, use the manual method:

### Method 1: Using Docusaurus Deploy Command

1. **Install gh-pages** (if not already installed):
   ```bash
   npm install --save-dev gh-pages
   ```

2. **Set up GitHub token** (if needed):
   - Go to GitHub → Settings → Developer settings → Personal access tokens
   - Create a token with `repo` permissions
   - Use it when prompted, or set as environment variable

3. **Deploy**:
   ```bash
   npm run deploy
   ```

4. **Enable GitHub Pages**:
   - Go to repository Settings → Pages
   - Source: `Deploy from a branch`
   - Branch: `gh-pages` → `/ (root)`
   - Save

### Method 2: Manual Build and Push

```bash
# Build the site
npm run build

# Install gh-pages if needed
npm install --save-dev gh-pages

# Deploy to gh-pages branch
npx gh-pages -d build -b gh-pages
```

Then enable GitHub Pages in Settings → Pages → Deploy from branch `gh-pages`.

## Common Issues

### Issue 1: "Workflow not found" or "No workflows"
**Solution**: Make sure `.github/workflows/deploy.yml` is committed and pushed to your repository.

### Issue 2: "Permission denied" in GitHub Actions
**Solution**: 
- Go to repository Settings → Actions → General
- Under "Workflow permissions", select "Read and write permissions"
- Check "Allow GitHub Actions to create and approve pull requests"
- Save

### Issue 3: Build fails in GitHub Actions
**Solution**: 
- Check the Actions tab for error messages
- Common issues: missing dependencies, Node version mismatch
- The workflow uses Node 20, make sure your `package.json` is compatible

### Issue 4: Site shows 404 after deployment
**Solutions**:
1. Wait 5-10 minutes (GitHub Pages can take time to propagate)
2. Clear browser cache or try incognito mode
3. Verify `baseUrl` in `docusaurus.config.js` matches repository name exactly
4. Check that GitHub Pages is enabled in Settings → Pages

### Issue 5: Wrong branch for deployment
**Solution**: 
- The workflow is set for `main` branch
- Either merge your current branch to `main`, or update the workflow file

## Verification Checklist

- [ ] `.github/workflows/deploy.yml` exists and is committed
- [ ] Code is pushed to GitHub (main branch or branch specified in workflow)
- [ ] GitHub Pages is enabled in repository Settings → Pages
- [ ] Source is set to "GitHub Actions" (for automatic) or "Deploy from a branch" (for manual)
- [ ] GitHub Actions workflow has run successfully (check Actions tab)
- [ ] Build completed without errors
- [ ] Waited 5-10 minutes after deployment
- [ ] `baseUrl` in `docusaurus.config.js` matches repository name: `/Physical-AI-and-Humanoid-Robotics/`

## Quick Test

After deployment, test these URLs:
- `https://Alisha-Kayani.github.io/Physical-AI-and-Humanoid-Robotics/`
- `https://Alisha-Kayani.github.io/Physical-AI-and-Humanoid-Robotics/docs/preface`

If both work, deployment is successful!

## Still Having Issues?

1. Check GitHub Actions logs in the **Actions** tab
2. Verify repository name matches exactly: `Physical-AI-and-Humanoid-Robotics`
3. Check that `organizationName` and `projectName` in `docusaurus.config.js` are correct
4. Make sure you have push access to the repository

