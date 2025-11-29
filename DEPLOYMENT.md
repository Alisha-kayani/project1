# Deployment Guide: Physical AI & Humanoid Robotics Book

This guide explains how to deploy your Docusaurus book to GitHub Pages.

## Prerequisites

1. **GitHub Repository**: Your code should be in a GitHub repository
   - Repository name: `Physical-AI-and-Humanoid-Robotics`
   - Organization/Username: `Alisha-Kayani`

2. **Git Installed**: Make sure Git is installed and configured

3. **GitHub Access**: You need push access to the repository

## Configuration Check

Before deploying, verify your `docusaurus.config.js` has the correct settings:

```javascript
url: 'https://Alisha-Kayani.github.io',
baseUrl: '/Physical-AI-and-Humanoid-Robotics/',
organizationName: 'Alisha-Kayani',
projectName: 'Physical-AI-and-Humanoid-Robotics',
deploymentBranch: 'gh-pages',
```

## Deployment Methods

### Method 1: Automatic Deployment with GitHub Actions (Recommended)

This method automatically deploys your site whenever you push to the main branch.

#### Step 1: Create GitHub Actions Workflow

1. Create the directory structure:
   ```bash
   mkdir -p .github/workflows
   ```

2. Create a workflow file (see `deploy.yml` example below)

#### Step 2: Push to GitHub

```bash
git add .
git commit -m "Add GitHub Actions deployment workflow"
git push origin main
```

The workflow will automatically:
- Build your site
- Deploy to the `gh-pages` branch
- Make your site available at: `https://Alisha-Kayani.github.io/Physical-AI-and-Humanoid-Robotics/`

### Method 2: Manual Deployment with Docusaurus CLI

This method uses Docusaurus's built-in deployment command.

#### Step 1: Install GitHub Pages Plugin (if not already installed)

```bash
npm install --save-dev gh-pages
```

#### Step 2: Configure Git Remote

Make sure your repository is connected to GitHub:

```bash
git remote -v
# Should show your GitHub repository URL
```

If not, add it:
```bash
git remote add origin https://github.com/Alisha-Kayani/Physical-AI-and-Humanoid-Robotics.git
```

#### Step 3: Set Up GitHub Token

You need a GitHub Personal Access Token with `repo` permissions:

1. Go to GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
2. Generate a new token with `repo` scope
3. Copy the token

#### Step 4: Configure Git Credentials

**Option A: Use GitHub CLI (Recommended)**
```bash
gh auth login
```

**Option B: Use Git Credential Helper**
```bash
git config --global credential.helper store
# Then use your token as password when prompted
```

**Option C: Set Environment Variable**
```bash
# Windows PowerShell
$env:GITHUB_TOKEN="your-token-here"

# Windows CMD
set GITHUB_TOKEN=your-token-here

# Linux/Mac
export GITHUB_TOKEN="your-token-here"
```

#### Step 5: Deploy

```bash
npm run deploy
```

This command will:
1. Build your site
2. Create/update the `gh-pages` branch
3. Push to GitHub
4. Your site will be live at: `https://Alisha-Kayani.github.io/Physical-AI-and-Humanoid-Robotics/`

### Method 3: Manual Build and Push

If the above methods don't work, you can manually build and push:

#### Step 1: Build the Site

```bash
npm run build
```

#### Step 2: Install gh-pages (if needed)

```bash
npm install --save-dev gh-pages
```

#### Step 3: Deploy to gh-pages Branch

```bash
npx gh-pages -d build -b gh-pages
```

Or manually:

```bash
# Create orphan branch
git checkout --orphan gh-pages
git rm -rf .

# Copy build files
cp -r build/* .

# Commit and push
git add .
git commit -m "Deploy site"
git push origin gh-pages --force

# Return to main branch
git checkout main
```

## Enable GitHub Pages

After deploying, enable GitHub Pages in your repository:

1. Go to your repository on GitHub
2. Click **Settings** → **Pages**
3. Under **Source**, select:
   - **Branch**: `gh-pages`
   - **Folder**: `/ (root)`
4. Click **Save**

Your site will be available at:
`https://Alisha-Kayani.github.io/Physical-AI-and-Humanoid-Robotics/`

## Troubleshooting

### Issue: Deployment fails with authentication error

**Solution**: Make sure you have a valid GitHub token configured. Use GitHub CLI or set up a Personal Access Token.

### Issue: Site shows 404 or blank page

**Solutions**:
1. Check that `baseUrl` in `docusaurus.config.js` matches your repository name
2. Wait a few minutes for GitHub Pages to propagate
3. Check the `gh-pages` branch exists and has content
4. Verify GitHub Pages is enabled in repository settings

### Issue: Links are broken

**Solution**: Make sure `baseUrl` is set correctly. It should be `/Physical-AI-and-Humanoid-Robotics/` (with trailing slash if using trailingSlash: true).

### Issue: Build fails locally

**Solution**: 
1. Clear cache: `npm run clear`
2. Reinstall dependencies: `rm -rf node_modules package-lock.json && npm install`
3. Check for errors: `npm run build`

## Updating Your Site

After making changes:

1. **If using GitHub Actions**: Just push to main branch
   ```bash
   git add .
   git commit -m "Update content"
   git push origin main
   ```

2. **If using manual deployment**: Run deploy command
   ```bash
   npm run deploy
   ```

## Custom Domain (Optional)

To use a custom domain:

1. Add a `CNAME` file in `static/` directory with your domain
2. Configure DNS settings for your domain
3. Update `url` in `docusaurus.config.js` to your custom domain
4. Redeploy

## Verification Checklist

- [ ] `docusaurus.config.js` has correct `url` and `baseUrl`
- [ ] GitHub repository exists and is accessible
- [ ] `gh-pages` branch exists with built files
- [ ] GitHub Pages is enabled in repository settings
- [ ] Site is accessible at the expected URL
- [ ] All links work correctly
- [ ] Navigation works (sidebar, navbar)
- [ ] Images and assets load correctly

## Next Steps

After successful deployment:

1. Share your book URL: `https://Alisha-Kayani.github.io/Physical-AI-and-Humanoid-Robotics/`
2. Set up automatic deployment (GitHub Actions recommended)
3. Consider adding analytics (Google Analytics, Plausible, etc.)
4. Set up custom domain if desired

## Additional Resources

- [Docusaurus Deployment Documentation](https://docusaurus.io/docs/deployment)
- [GitHub Pages Documentation](https://docs.github.com/en/pages)
- [GitHub Actions Documentation](https://docs.github.com/en/actions)

