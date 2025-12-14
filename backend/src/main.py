from fastapi import FastAPI
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = FastAPI(
    title="AI-Powered Physical AI & Humanoid Robotics Textbook API",
    description="API for the AI-powered textbook platform that teaches Physical AI & Humanoid Robotics with RAG chatbot, personalization, and Urdu translation capabilities",
    version="1.0.0"
)

@app.get("/")
def read_root():
    return {"message": "Welcome to the AI-Powered Physical AI & Humanoid Robotics Textbook API"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)