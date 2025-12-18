from setuptools import setup, find_packages

setup(
    name="ai_textbook_backend",
    version="1.0.0",
    packages=find_packages(where="backend/src"),
    package_dir={"": "backend/src"},
    install_requires=[
        "fastapi==0.104.1",
        "uvicorn[standard]==0.24.0",
        "openai==1.3.8",
        "better-auth==0.0.1b11",
        "qdrant-client==1.8.0",
        "asyncpg==0.29.0",
        "sqlalchemy==2.0.23",
        "pydantic==2.5.0",
        "python-multipart==0.0.6",
        "python-dotenv==1.0.0",
        "pytest==7.4.3",
        "httpx==0.25.2",
        "passlib[bcrypt]==1.7.4",
        "pyjwt==2.8.0",
        "gunicorn==21.2.0",
    ],
)