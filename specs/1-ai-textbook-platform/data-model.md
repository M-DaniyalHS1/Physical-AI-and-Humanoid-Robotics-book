# Data Model: AI-Powered Physical AI & Humanoid Robotics Textbook

## Overview
This document defines the data models for the AI-Powered Physical AI & Humanoid Robotics Textbook platform, based on the entities identified in the feature specification.

## Entity Models

### User
Represents a registered user with background information, personalization settings, and authentication tokens.

**Fields:**
- `id` (string): Unique identifier for the user
- `email` (string): User's email address for authentication
- `password_hash` (string): Hashed password for security
- `software_experience` (string): User's software background level (beginner/intermediate/advanced)
- `hardware_experience` (string): User's hardware/robotics experience level
- `math_physics_level` (string): User's math and physics proficiency
- `learning_goals` (string): User's learning objectives
- `personalization_settings` (object): Per-chapter personalization preferences
- `created_at` (datetime): Account creation timestamp
- `updated_at` (datetime): Last update timestamp
- `last_login` (datetime): Last login timestamp

**Validation Rules:**
- Email must be valid and unique
- Password must meet security requirements
- Experience levels must be from predefined options
- Created_at and updated_at are automatically managed

**Relationships:**
- One-to-many with Progress (user can have progress for multiple chapters)
- One-to-many with ChatSession (user can have multiple chat sessions)

### Book Content
Represents the textbook chapters, modules, sections, and learning materials organized in the 4-module curriculum, stored as Markdown files with automatic indexing to vector database for RAG functionality.

**Fields:**
- `id` (string): Unique identifier for the content
- `title` (string): Title of the chapter/module
- `module` (string): Module name (ROS 2, Gazebo & Unity, NVIDIA Isaac, VLA)
- `chapter_number` (integer): Sequential number within the module
- `content_type` (string): Type of content (text, video, diagram, lab, etc.)
- `content` (string): Markdown content
- `version` (string): Version number for tracking changes
- `vector_id` (string): Reference to vector in Qdrant database
- `created_at` (datetime): Creation timestamp
- `updated_at` (datetime): Last update timestamp
- `authors` (array): List of author IDs (human and AI)
- `learning_objectives` (array): List of learning objectives for the chapter

**Validation Rules:**
- Content must be in valid Markdown format
- Module must be one of the 4 specified modules
- Version must follow semantic versioning
- Vector_id must correspond to an entry in the vector database

**Relationships:**
- One-to-many with ContentMetadata (metadata for various personalization levels)
- One-to-many with Translation (translations in different languages)

### Chat Session
Represents a conversation between a user and the AI chatbot with context from selected text.

**Fields:**
- `id` (string): Unique identifier for the chat session
- `user_id` (string): Reference to the user
- `session_start` (datetime): When the session started
- `session_end` (datetime): When the session ended (null if active)
- `selected_text` (string): Text selected in the chapter (null if not applicable)
- `mode` (string): Chat mode ('general' or 'selected-text-only')
- `is_active` (boolean): Whether the session is currently active

**Validation Rules:**
- user_id must reference a valid user
- mode must be 'general' or 'selected-text-only'
- session_start must be before session_end if session_end is set

**Relationships:**
- One-to-many with ChatMessage (chat session contains multiple messages)

### Chat Message
Represents individual messages in a chat session.

**Fields:**
- `id` (string): Unique identifier for the message
- `session_id` (string): Reference to the chat session
- `sender_type` (string): Who sent the message ('user' or 'ai')
- `content` (string): The message content
- `timestamp` (datetime): When the message was sent
- `context_used` (string): Context from book content used to generate response

**Validation Rules:**
- session_id must reference a valid active chat session
- sender_type must be 'user' or 'ai'
- Content must not be empty

**Relationships:**
- Belongs to one ChatSession
- References one BookContent (through context_used)

### Personalization Profile
Represents user-specific settings that adapt content based on background and preferences.

**Fields:**
- `id` (string): Unique identifier for the profile
- `user_id` (string): Reference to the user
- `chapter_id` (string): Reference to the specific chapter
- `content_level` (string): Content depth (beginner/intermediate/advanced)
- `example_preference` (string): Type of examples preferred (theoretical/practical)
- `update_context` (string): Context for when this personalization was set
- `created_at` (datetime): Creation timestamp
- `updated_at` (datetime): Last update timestamp

**Validation Rules:**
- user_id must reference a valid user
- chapter_id must reference a valid chapter
- content_level must be one of the specified options

**Relationships:**
- Belongs to one User
- References one BookContent

### Translation Cache
Represents cached translations of content in Urdu to improve performance.

**Fields:**
- `id` (string): Unique identifier for the translation
- `content_id` (string): Reference to the original book content
- `target_language` (string): Target language code (e.g., 'ur' for Urdu)
- `translated_content` (string): Translated content in target language
- `translation_method` (string): How translation was created ('ai', 'human', 'ai_with_review')
- `quality_score` (number): Quality rating of the translation (0-1)
- `created_at` (datetime): Creation timestamp
- `updated_at` (datetime): Last update timestamp

**Validation Rules:**
- content_id must reference a valid book content
- target_language must be a valid language code
- quality_score must be between 0 and 1

**Relationships:**
- References one BookContent

### Progress
Represents user progress including chapters read and exercises completed.

**Fields:**
- `id` (string): Unique identifier for the progress record
- `user_id` (string): Reference to the user
- `content_id` (string): Reference to the book content
- `progress_percentage` (number): How much of the content has been consumed (0-100)
- `time_spent_seconds` (integer): Time spent on the content in seconds
- `last_accessed` (datetime): When the content was last accessed
- `completed_exercises` (array): IDs of completed exercises
- `bookmark_position` (integer): Bookmark position in the content (character offset)
- `created_at` (datetime): Creation timestamp
- `updated_at` (datetime): Last update timestamp

**Validation Rules:**
- user_id must reference a valid user
- content_id must reference a valid book content
- progress_percentage must be between 0 and 100
- bookmark_position must be non-negative and less than content length

**Relationships:**
- Belongs to one User
- References one BookContent

### Content Metadata
Represents metadata for different personalization levels of the same content.

**Fields:**
- `id` (string): Unique identifier for the metadata
- `content_id` (string): Reference to the book content
- `personalization_level` (string): Target personalization level (beginner/intermediate/advanced)
- `adjusted_content` (string): Content adjusted for the specific level
- `adjusted_examples` (array): Examples tailored for this level
- `difficulty_rating` (number): Numeric rating of difficulty (1-5)
- `estimated_time_minutes` (integer): Estimated time to complete in minutes

**Validation Rules:**
- content_id must reference a valid book content
- personalization_level must be one of the specified options
- difficulty_rating must be between 1 and 5

**Relationships:**
- References one BookContent

## State Transitions

### User Session States
- Registered → Active (user logs in for the first time)
- Active → Inactive (user doesn't log in for extended period)
- Active → Deleted (user requests account deletion)

### Content States
- Draft → Published (content is approved and made available)
- Published → Updated (content is modified and republished)
- Published → Archived (content is deprecated but kept for reference)

### Chat Session States
- Created → Active (user starts interacting)
- Active → Ended (session expires or user ends it)
- Ended → Archived (session data is preserved for analytics)