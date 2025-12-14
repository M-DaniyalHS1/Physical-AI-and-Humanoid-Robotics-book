# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Prerequisites

- Node.js 18+ for Docusaurus frontend
- Python 3.11+ for backend services
- PostgreSQL-compatible database (Neon Serverless Postgres recommended)
- Vector database (Qdrant Cloud recommended)
- OpenAI API key for chatbot functionality
- Better Auth account for authentication

## Initial Setup

### 1. Clone and Initialize the Repository

```bash
git clone <repository-url>
cd physical-ai-textbook
```

### 2. Frontend Setup (Docusaurus)

```bash
cd frontend
npm install
```

### 3. Backend Setup

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 4. Environment Configuration

Create `.env` files in both frontend and backend directories:

**Frontend (.env):**
```env
REACT_APP_API_BASE_URL=http://localhost:8000
REACT_APP_OPENAI_API_KEY=your_openai_api_key
```

**Backend (.env):**
```env
DATABASE_URL=your_neon_postgres_connection_string
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
OPENAI_API_KEY=your_openai_api_key
BETTER_AUTH_SECRET=your_auth_secret
BETTER_AUTH_URL=http://localhost:3000
```

## Running the Application

### 1. Start the Backend

```bash
cd backend
python -m uvicorn main:app --reload --port 8000
```

### 2. Start the Frontend

```bash
cd frontend
npm run start
```

The application will be available at `http://localhost:3000`

## Key Components

### 1. Content Structure

The textbook content is organized under `frontend/docs/` with four main modules:
- `module-1-ros2/` - The Robotic Nervous System (ROS 2)
- `module-2-gazebo-unity/` - The Digital Twin (Gazebo & Unity)
- `module-3-nvidia-isaac/` - The AI-Robot Brain (NVIDIA Isaac™)
- `module-4-vla/` - Vision-Language-Action (VLA)

### 2. AI Components

- Chatbot API: `/api/v1/chatbot/query` - Handles RAG queries
- Translation API: `/api/v1/translation/{content_id}` - Provides Urdu translation
- Personalization API: `/api/v1/content/{content_id}/personalize` - Adapts content to user background

### 3. User Authentication

Uses Better Auth for user registration and login, collecting background information during signup:
- Software background
- Hardware/robotics experience
- Math and physics level
- Learning goals

## Adding New Content

1. Create a new markdown file in the appropriate module directory
2. Follow the content template with learning objectives, explanations, diagrams, and AI-QA hooks
3. Update `sidebars.js` to include the new content in the navigation
4. The content will be automatically indexed for the RAG chatbot

## Testing

### Frontend Testing
```bash
npm run test
npm run e2e  # End-to-end tests
```

### Backend Testing
```bash
python -m pytest tests/
```

## Deployment

### GitHub Pages (Frontend)
```bash
npm run build
npm run deploy
```

### Backend Deployment
Deploy the FastAPI backend to your preferred cloud platform, ensuring all environment variables are properly configured.

## Troubleshooting

### Common Issues

1. **Chatbot not responding**: Verify OpenAI API key is correct and Qdrant vector store is properly populated
2. **Authentication fails**: Check Better Auth configuration and ensure API URLs are correctly set
3. **Translation unavailable**: Ensure translation service is running and API keys are valid
4. **Personalization not working**: Verify user profile is properly set with background information

### Useful Commands

- `npm run serve` - Serve the built static files locally (for testing production build)
- `python -m scripts.populate_qdrant` - Rebuild vector store from content
- `npm run check-links` - Check for broken links in documentation