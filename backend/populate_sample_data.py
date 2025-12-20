"""
Script to populate the database with sample content for testing
"""
import sqlite3
import json
from datetime import datetime
import uuid

def populate_sample_data():
    conn = sqlite3.connect('ai_textbook.db')
    cursor = conn.cursor()
    
    # Sample content data
    sample_contents = [
        {
            "id": "intro-ros",
            "title": "Introduction to ROS 2",
            "module": "ROS 2",
            "chapter_number": 1,
            "content_type": "text",
            "content": "Robot Operating System (ROS) is flexible framework for writing robot software...",
            "version": "1.0.0",
            "authors": json.dumps(["Dr. Jane Smith", "Prof. John Doe"]),
            "learning_objectives": json.dumps([
                "Understand ROS 2 architecture",
                "Learn about nodes and topics",
                "Practice with basic commands"
            ])
        },
        {
            "id": "gazebo-simulation",
            "title": "Gazebo Simulation Environment",
            "module": "Gazebo & Unity",
            "chapter_number": 2,
            "content_type": "text",
            "content": "Gazebo is a robotics simulator that provides realistic physics simulation...",
            "version": "1.0.0",
            "authors": json.dumps(["Dr. Alice Johnson"]),
            "learning_objectives": json.dumps([
                "Set up Gazebo environment",
                "Create simulation worlds",
                "Integrate with ROS"
            ])
        },
        {
            "id": "isaac-navigation",
            "title": "Navigation with NVIDIA Isaac",
            "module": "NVIDIA Isaac",
            "chapter_number": 3,
            "content_type": "text",
            "content": "NVIDIA Isaac is a robotics platform that accelerates development of autonomous machines...",
            "version": "1.0.0",
            "authors": json.dumps(["Dr. Bob Wilson", "Eng. Sarah Lee"]),
            "learning_objectives": json.dumps([
                "Install Isaac SDK",
                "Build navigation applications",
                "Deploy to hardware"
            ])
        },
        {
            "id": "vla-overview",
            "title": "Vision-Language-Action Models",
            "module": "VLA",
            "chapter_number": 4,
            "content_type": "text",
            "content": "Vision-Language-Action (VLA) models enable robots to understand and execute complex tasks...",
            "version": "1.0.0",
            "authors": json.dumps(["Dr. Chris Brown"]),
            "learning_objectives": json.dumps([
                "Understand VLA architecture",
                "Implement basic VLA models",
                "Evaluate model performance"
            ])
        }
    ]
    
    # Insert sample data
    for content in sample_contents:
        try:
            cursor.execute("""
                INSERT OR REPLACE INTO book_content 
                (id, title, module, chapter_number, content_type, content, version, 
                 vector_id, created_at, updated_at, authors, learning_objectives)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                content["id"],
                content["title"],
                content["module"],
                content["chapter_number"],
                content["content_type"],
                content["content"],
                content["version"],
                str(uuid.uuid4()),  # vector_id
                datetime.utcnow().isoformat(),
                datetime.utcnow().isoformat(),
                content["authors"],
                content["learning_objectives"]
            ))
        except sqlite3.Error as e:
            print(f"Error inserting content {content['id']}: {e}")
    
    # Also insert some sample metadata
    sample_metadata = [
        {
            "id": f"meta-{content['id']}-beginner",
            "content_id": content["id"],
            "personalization_level": "beginner",
            "difficulty_score": 2.5,
            "estimated_time_minutes": 30,
            "prerequisites": json.dumps([]),
            "keywords": json.dumps(content["learning_objectives"][:2]),
            "created_at": datetime.utcnow().isoformat(),
            "updated_at": datetime.utcnow().isoformat()
        } for content in sample_contents
    ] + [
        {
            "id": f"meta-{content['id']}-intermediate",
            "content_id": content["id"],
            "personalization_level": "intermediate",
            "difficulty_score": 5.0,
            "estimated_time_minutes": 45,
            "prerequisites": json.dumps([obj for obj in content["learning_objectives"][:1]]),
            "keywords": json.dumps(content["learning_objectives"]),
            "created_at": datetime.utcnow().isoformat(),
            "updated_at": datetime.utcnow().isoformat()
        } for content in sample_contents
    ]
    
    for metadata in sample_metadata:
        try:
            cursor.execute("""
                INSERT OR REPLACE INTO content_metadata
                (id, content_id, personalization_level, difficulty_score, estimated_time_minutes,
                 prerequisites, keywords, created_at, updated_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                metadata["id"],
                metadata["content_id"],
                metadata["personalization_level"],
                metadata["difficulty_score"],
                metadata["estimated_time_minutes"],
                metadata["prerequisites"],
                metadata["keywords"],
                metadata["created_at"],
                metadata["updated_at"]
            ))
        except sqlite3.Error as e:
            print(f"Error inserting metadata {metadata['id']}: {e}")
    
    conn.commit()
    conn.close()
    
    print(f"Successfully inserted {len(sample_contents)} sample content items and {len(sample_metadata)} metadata records")

if __name__ == "__main__":
    populate_sample_data()