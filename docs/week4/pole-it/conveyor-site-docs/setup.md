# Setup Instructions for Conveyor-Site Project

## Prerequisites

Before you begin, ensure you have the following installed on your local machine:

- **Node.js**: Version 14 or higher. You can download it from [Node.js official website](https://nodejs.org/).
- **npm**: Node package manager, which comes bundled with Node.js.
- **Redis**: A Redis server or access to a cloud-based Redis service (e.g., Upstash).

## Installation Steps

1. **Clone the Repository**

   Open your terminal and run the following command to clone the repository:

   ```
   git clone https://github.com/PatriceDAGBE/conveyor-site.git
   ```

2. **Navigate to the Project Directory**

   Change your working directory to the cloned project:

   ```
   cd conveyor-site
   ```

3. **Install Dependencies**

   Run the following command to install the required dependencies:

   ```
   pnpm install
   ```

4. **Set Up Environment Variables**

   Create a `.env.local` file in the root of the project and add your Redis configuration:

   ```
   UPSTASH_REDIS_REST_URL=https://your-redis-url
   UPSTASH_REDIS_REST_TOKEN=your-redis-token
   ```

   Replace `https://your-redis-url` and `your-redis-token` with your actual Redis connection details.

5. **Run the Application**

   Start the development server with the following command:

   ```
   npm run dev
   ```

   The application should now be running on `http://localhost:3000`.

## Accessing the Application

Open your web browser and navigate to `http://localhost:3000` to access the Conveyor-Site application.

## Additional Notes

- Ensure that your Redis server is running and accessible.
- For any issues during setup, refer to the [Troubleshooting](troubleshooting.md) section of the documentation.