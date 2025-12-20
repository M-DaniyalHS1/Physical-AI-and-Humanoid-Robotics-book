"""
Script to update the content_metadata table with missing columns
"""
import sqlite3
from datetime import datetime

def update_content_metadata_table():
    conn = sqlite3.connect('ai_textbook.db')
    cursor = conn.cursor()
    
    # Check if the columns exist
    cursor.execute('PRAGMA table_info(content_metadata);')
    columns = [column[1] for column in cursor.fetchall()]
    
    # Add created_at column if it doesn't exist
    if 'created_at' not in columns:
        try:
            cursor.execute('ALTER TABLE content_metadata ADD COLUMN created_at DATETIME')
            print("Added created_at column to content_metadata table")
        except sqlite3.Error as e:
            print(f"Error adding created_at column: {e}")
    
    # Add updated_at column if it doesn't exist
    if 'updated_at' not in columns:
        try:
            cursor.execute('ALTER TABLE content_metadata ADD COLUMN updated_at DATETIME')
            print("Added updated_at column to content_metadata table")
        except sqlite3.Error as e:
            print(f"Error adding updated_at column: {e}")
    
    # Set default values for existing records
    try:
        cursor.execute("""
            UPDATE content_metadata 
            SET created_at = ?, updated_at = ?
            WHERE created_at IS NULL OR updated_at IS NULL
        """, (datetime.utcnow().isoformat(), datetime.utcnow().isoformat()))
        print(f"Updated {cursor.rowcount} records with default timestamps")
    except sqlite3.Error as e:
        print(f"Error updating records: {e}")
    
    conn.commit()
    conn.close()
    
    print("Content metadata table update completed")

if __name__ == "__main__":
    update_content_metadata_table()