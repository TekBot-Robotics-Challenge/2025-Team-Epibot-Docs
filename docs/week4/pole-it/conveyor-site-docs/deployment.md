# Deployment Instructions for Conveyor-Site

This document provides detailed instructions on how we deploy the Conveyor-Site project to a production environment.

## Prerequisites

Before deploying the application, we ensure that we have:

- A Vercel account (recommended deployment platform)
- Access to a Redis instance (e.g., Upstash)
- Your environment variables ready

[![Vercel Homepage](/week4/images/IT/Website/vercel-home.png)](/week4/images/IT/Website/vercel-home.png)

## Deployment Steps

### Deploying to Vercel

1. **Push your code to GitHub, GitLab, or Bitbucket.**  
   We push our code on GitHub from `conveyor-site` repository

   ```bash
   git add .
   git commit -m "commit"
   git push
   ```

2. **Import your repository into Vercel** at [https://vercel.com/import](https://vercel.com/import).  
   [![Vercel Homepage](/week4/images/IT/Website/vercel-import.png)](/week4/images/IT/Website/vercel-import.png)

3. **Configure environment variables** as described above.  

4. **Deploy**: Vercel will automatically build and deploy your application.  
   After deployment, our app is accessible at the provided Vercel URL (e.g., `https://conveyor-site.vercel.app`). 

## Configuration

1. **Environment Variables**:  
   In your Vercel project dashboard, go to **Settings > Environment Variables** and add:

   - `UPSTASH_REDIS_REST_URL` = `<your_upstash_redis_url>`
   - `UPSTASH_REDIS_REST_TOKEN` = `<your_upstash_redis_token>`

   [![Vercel Homepage](/week4/images/IT/Website/vercel-configuration.png)](/week4/images/IT/Website/vercel-configuration.png)

   Replace the values with your actual Upstash Redis credentials.

2. **Production Settings**:  
   No special changes are needed for `next.config.mjs` when deploying to Vercel. Vercel automatically optimizes your Next.js app for production.


## Conclusion

By following these steps, you can easily deploy Conveyor-Site to production using Vercel. For troubleshooting, refer to the troubleshooting document or Vercel’s documentation.

---