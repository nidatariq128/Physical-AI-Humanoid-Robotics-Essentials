# Physical AI Humanoid Robotics Documentation

This is the documentation site for the Physical AI Humanoid Robotics project, built with Docusaurus.

## Deployment to Vercel

This project is configured for deployment on Vercel. Follow these steps to deploy:

### 1. Prepare Your Repository
1. Push this code to a GitHub repository
2. Make sure all changes are committed

### 2. Deploy via Vercel Dashboard
1. Go to [Vercel](https://vercel.com)
2. Sign in with your GitHub account
3. Click "New Project" and select your GitHub repository
4. Vercel will automatically detect this is a Docusaurus project
5. Use the following build settings:
   - Build Command: `npm run build` or `npm run vercel-build`
   - Output Directory: `build`
   - Install Command: `npm install`
6. Click "Deploy"

### 3. Deploy via Vercel CLI (Alternative)
1. Install Vercel CLI: `npm install -g vercel`
2. Login: `vercel login`
3. Deploy: `vercel`

### Configuration Notes
- The site is configured to work at the root path (`/`) for Vercel deployment
- Custom domain can be added in the Vercel dashboard after deployment
- Environment variables can be configured in the Vercel dashboard if needed

## Local Development

```bash
npm install
npm start
```

## Building for Production

```bash
npm run build
```

The built files will be in the `build` directory and are ready for deployment.

## Customization

- Edit `docusaurus.config.js` to change site configuration
- Edit CSS in `src/css/custom.css` to customize styling
- Add documentation in the `docs` directory
- Customize pages in the `src/pages` directory