# GitHub Pages Deployment Configuration

This project is configured to deploy to GitHub Pages using GitHub Actions.

## Setup Instructions

1. Ensure the workflow file `.github/workflows/gh-pages.yml` is present in your repository
2. Make sure your `docusaurus.config.js` has the correct:
   - `url`: Set to your GitHub Pages URL (e.g., `https://panaversity.github.io`)
   - `baseUrl`: Set to your project name (e.g., `/ai-textbook-platform`)
   - `organizationName`: Your GitHub username or organization name
   - `projectName`: Your repository name

3. In your GitHub repository:
   - Go to Settings > Pages
   - Under "Source", select "Deploy from a branch"
   - Select your main branch and `/docs/build` as the folder

4. Push your changes to the main branch to trigger the deployment

## Workflow Details

The GitHub Actions workflow:
- Runs on pushes to the main branch
- Sets up Node.js environment
- Installs dependencies from the frontend directory
- Builds the Docusaurus site
- Deploys the built site to GitHub Pages

## Troubleshooting

- If your site doesn't load properly, check that the `baseUrl` in `docusaurus.config.js` matches your GitHub Pages subdirectory
- Ensure that your custom domain settings (if any) are properly configured in GitHub repository settings