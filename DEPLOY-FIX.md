# Fix: GIT_USER Environment Variable Error

## The Problem
`npm run deploy` requires authentication. You need to set up either:
- **Option 1**: GIT_USER and GIT_PASS (GitHub token)
- **Option 2**: USE_SSH=true (if you have SSH keys)
- **Option 3**: Use manual gh-pages deployment (easiest)

## Solution 1: Use GitHub Token (Recommended for Windows)

### Step 1: Create GitHub Personal Access Token

1. Go to: https://github.com/settings/tokens
2. Click **Generate new token** → **Generate new token (classic)**
3. Name it: "Docusaurus Deploy"
4. Select scope: **repo** (full control of private repositories)
5. Click **Generate token**
6. **Copy the token immediately** (you won't see it again!)

### Step 2: Set Environment Variables (Windows PowerShell)

Run these commands in PowerShell:

```powershell
# Set your GitHub username
$env:GIT_USER="Alisha-Kayani"

# Set your GitHub token (replace YOUR_TOKEN with the actual token)
$env:GIT_PASS="YOUR_TOKEN_HERE"

# Now deploy
npm run deploy
```

### Step 3: Deploy

```powershell
npm run deploy
```

## Solution 2: Use SSH (If You Have SSH Keys Set Up)

If you already have SSH keys configured with GitHub:

```powershell
$env:USE_SSH="true"
npm run deploy
```

## Solution 3: Manual Deployment (Easiest - No Environment Variables)

This method doesn't require environment variables:

```powershell
# Build the site
npm run build

# Deploy using gh-pages directly
npx gh-pages -d build -b gh-pages
```

**Note**: For this method, you'll need to authenticate when prompted, or set up a GitHub token in Git credential helper.

## Solution 4: Use GitHub Actions (Best for Automatic Deployment)

Since you already have the workflow file, the easiest is to use GitHub Actions:

1. **Push your code**:
   ```powershell
   git add .
   git commit -m "Ready for deployment"
   git push origin 001-docusaurus-config
   ```

2. **Enable GitHub Pages**:
   - Go to: https://github.com/Alisha-Kayani/Physical-AI-and-Humanoid-Robotics/settings/pages
   - Source: **GitHub Actions**
   - Save

3. **Trigger workflow**:
   - Go to: https://github.com/Alisha-Kayani/Physical-AI-and-Humanoid-Robotics/actions
   - Click "Deploy to GitHub Pages"
   - Click "Run workflow"

This method doesn't require any local environment variables!

## Quick Fix Script (PowerShell)

Save this as `deploy.ps1`:

```powershell
# Option 1: Using Token
$env:GIT_USER="Alisha-Kayani"
$env:GIT_PASS="YOUR_TOKEN_HERE"
npm run deploy

# Option 2: Using SSH
# $env:USE_SSH="true"
# npm run deploy

# Option 3: Manual
# npm run build
# npx gh-pages -d build -b gh-pages
```

Then run: `.\deploy.ps1`

## Recommended Approach

**For now**: Use **Solution 3 (Manual)** or **Solution 4 (GitHub Actions)**

**Long-term**: Set up **Solution 4 (GitHub Actions)** - it's automatic and doesn't require local setup!

