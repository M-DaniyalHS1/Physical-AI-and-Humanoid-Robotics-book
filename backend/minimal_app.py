"""
Minimal application to test if basic requests work
"""
from fastapi import FastAPI
import time

app = FastAPI()

@app.get("/")
def read_root():
    return {"message": "Minimal app is running", "status": "healthy", "timestamp": time.time()}

@app.get("/health")
def health_check():
    return {"status": "healthy", "version": "1.0.0", "timestamp": time.time()}

@app.get("/test")
def test_endpoint():
    return {"status": "working", "message": "Request processed successfully"}