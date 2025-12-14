# AI-Powered Physical AI & Humanoid Robotics Textbook

This is an AI-native textbook platform that teaches Physical AI & Humanoid Robotics with RAG chatbot, personalization, and Urdu translation capabilities.

## Configuration

To run this project, you need to set up the following environment variables in a `.env` file in the root directory:

```env
# API Keys and Configuration
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
DATABASE_URL=your_neon_postgres_connection_string_here
JWT_SECRET=your_jwt_secret_key_here

# Rate Limiting
CHAT_RATE_LIMIT=10
TRANSLATION_RATE_LIMIT=50
CONTENT_RATE_LIMIT=100
```

### Required Services

You'll need to set up accounts with these services:

1. **OpenAI**: For the RAG chatbot functionality
2. **Qdrant Cloud**: For vector storage for the RAG system
3. **Neon Serverless Postgres**: For user data storage
4. **GitHub Pages**: For frontend deployment (configured via GitHub Actions)

## Running the Project

### Backend (Python/FastAPI)

1. Navigate to the `backend` directory
2. Install dependencies: `pip install -r requirements.txt`
3. Run the server: `python -m src.main`

### Frontend (Docusaurus)

1. Navigate to the `docs` directory
2. Install dependencies: `npm install`
3. Run locally: `npm start`

## Architecture

The project follows a microservices architecture:
- Backend: FastAPI server for API endpoints, authentication, and AI integration
- Frontend: Docusaurus-based static site deployed to GitHub Pages
- Data: PostgreSQL for user data, Qdrant for vector storage of textbook content