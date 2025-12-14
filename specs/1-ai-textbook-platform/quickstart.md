# Quickstart Guide: AI-Powered Physical AI & Humanoid Robotics Textbook

## Overview
This guide will help you set up and run the AI-Powered Physical AI & Humanoid Robotics Textbook platform locally for development and testing.

## Prerequisites
- Python 3.11 installed
- Node.js 18+ installed
- Access to OpenAI Agent SDK and Chainkit
- Access to Qdrant Cloud account
- Access to Neon Serverless Postgres account

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd ai-textbook-platform
```

### 2. Backend Setup
1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Set up environment variables:
   Create a `.env` file with the following:
   ```env
   OPENAI_API_KEY=your_openai_api_key
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   DATABASE_URL=your_neon_postgres_connection_string
   JWT_SECRET=your_jwt_secret_key
   ```

5. Run the backend:
   ```bash
   uvicorn main:app --reload
   ```

### 3. Frontend Setup
1. Navigate to the frontend directory:
   ```bash
   cd frontend  # or docs if using Docusaurus directly
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Set up environment variables:
   Create a `.env` file with the following:
   ```env
   VITE_API_BASE_URL=http://localhost:8000
   ```

4. Run the development server:
   ```bash
   npm run dev
   ```

### 4. Docusaurus Setup (if using Docusaurus)
1. Navigate to the docs directory:
   ```bash
   cd docs
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Run the development server:
   ```bash
   npm start
   ```

## Initial Configuration

### 1. Content Setup
1. Add textbook content to the `docs/` directory following the Docusaurus structure
2. Each module (ROS 2, Gazebo & Unity, NVIDIA Isaac, VLA) should have its own directory
3. Ensure all content is in Markdown format

### 2. Index Content for RAG
1. Run the content indexing script to add your textbook content to Qdrant:
   ```bash
   python scripts/index_content.py
   ```

### 3. Database Migrations
1. Run database migrations:
   ```bash
   alembic upgrade head
   ```

## Running the Application

### Development
- Start the backend: `uvicorn main:app --reload`
- Start the frontend: `npm run dev` or `npm start` (for Docusaurus)

### Production Deployment
1. Build the frontend: `npm run build`
2. Deploy to GitHub Pages (for static content)
3. Deploy backend services to serverless/container platform (Fly.io, Render, etc.)

## API Endpoints

The backend serves the following key API endpoints:

- `POST /auth/signup` - User registration
- `POST /auth/login` - User login
- `GET /content` - Get all textbook content
- `GET /content/{id}` - Get specific content
- `POST /content/{id}/translate` - Translate content to Urdu
- `GET /content/{id}/personalize` - Get personalized content
- `POST /chat/sessions` - Create a new chat session
- `POST /chat/sessions/{sessionId}/messages` - Send message to AI
- `GET /progress` - Get user's progress
- `POST /progress/{contentId}` - Update user's progress

## Configuration Options

### Personalization
Users can customize their learning experience based on their background knowledge. The system supports:
- Beginner, intermediate, and advanced content levels
- Different types of examples (theoretical vs practical)
- Context-specific difficulty adjustments

### Translation
The platform supports on-demand translation to Urdu. The translation feature:
- Caches translations to improve performance
- Supports human review of AI-generated translations
- Tracks translation quality metrics

## Troubleshooting

### Common Issues

1. **Qdrant Connection Issues**: Verify your Qdrant URL and API key in environment variables
2. **Database Connection Issues**: Ensure your Neon Postgres connection string is correct
3. **API Rate Limits**: The system implements rate limiting (10/50/100 requests per minute for different endpoints)
4. **Authentication Issues**: Check that JWT secret is configured correctly

### Performance Considerations
- AI responses may take 2-4 seconds as specified in the requirements
- Translation requests are limited to 50 per minute to manage costs
- Content access is limited to 100 requests per minute to handle scale

## Next Steps

1. Customize the Docusaurus theme to match your branding
2. Add more content modules as needed
3. Implement additional personalization options
4. Enhance the translation workflow with human review capabilities
5. Add analytics to track user engagement and content performance