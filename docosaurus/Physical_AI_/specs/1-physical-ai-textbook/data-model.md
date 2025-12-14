# Data Model: Physical AI & Humanoid Robotics Textbook

## Entity: User
**Description**: Represents a registered user with background information and personalization settings

**Fields**:
- id (string, UUID): Unique identifier for the user
- email (string): User's email address
- password_hash (string): Hashed password for authentication
- software_background (string): User's software experience level
- hardware_background (string): User's hardware/robotics experience
- math_physics_level (string): Proficiency in math and physics
- learning_goals (string): User's learning objectives
- created_at (datetime): Account creation timestamp
- updated_at (datetime): Last profile update timestamp
- preferences (json): Personalization preferences object

**Relationships**:
- One User to Many UserProgress
- One User to Many ChatSession
- One User to Many TranslationHistory

**Validation**:
- Email must be valid email format
- Background fields must be from predefined options
- Must provide at least one learning goal

## Entity: BookContent
**Description**: Represents the textbook chapters, modules, sections, and learning materials organized in the 4-module curriculum

**Fields**:
- id (string, UUID): Unique identifier for the content
- module_id (string): Associated module identifier (ROS2, GazeboUnity, NVIDIAIsaac, VLA)
- chapter_number (integer): Sequential chapter number within module
- title (string): Chapter title
- content (text): Main content in markdown format
- learning_objectives (json): List of learning objectives
- diagrams (json): List of diagram/image references
- ai_qa_hooks (json): Embedded questions and answers for AI
- version (string): Content version
- created_at (datetime): Creation timestamp
- updated_at (datetime): Last update timestamp
- author (string): Content author (human or AI identifier)

**Relationships**:
- One BookContent to Many UserProgress
- One BookContent to Many ChatSession (as context source)

**Validation**:
- Content must be at least 500 words
- Must have at least one learning objective
- Module ID must be one of the four core modules

## Entity: UserProgress
**Description**: Tracks user progress including chapters read, exercises completed, and engagement metrics

**Fields**:
- id (string, UUID): Unique identifier for the progress record
- user_id (string, UUID): Reference to User
- content_id (string, UUID): Reference to BookContent
- status (string): Current status (not_started, in_progress, completed)
- progress_percentage (integer): Percentage of content consumed
- time_spent_seconds (integer): Time spent on this content
- completed_exercises (json): List of completed exercises
- personalization_adjustments (json): Adjustments made based on user background
- last_accessed_at (datetime): Timestamp of last access
- completed_at (datetime): Timestamp when marked as completed

**Relationships**:
- One User to Many UserProgress
- One BookContent to Many UserProgress

**Validation**:
- Progress percentage must be between 0 and 100
- Status must be one of the defined values
- User and content IDs must reference valid entities

## Entity: ChatSession
**Description**: Represents a conversation between a user and the AI chatbot with context from selected text

**Fields**:
- id (string, UUID): Unique identifier for the chat session
- user_id (string, UUID): Reference to User
- content_id (string, UUID): Reference to BookContent providing context
- selected_text (text): The text selected by user for context
- query (text): The user's question
- response (text): The AI's response
- timestamp (datetime): When the query was made
- relevance_score (float): How relevant the response was (0-1)
- feedback (text): User feedback on the response
- token_usage (json): Details of token usage for the interaction

**Relationships**:
- One User to Many ChatSession
- One BookContent to Many ChatSession

**Validation**:
- Query must not be empty
- Relevance score must be between 0 and 1
- Timestamp must be in the past

## Entity: PersonalizationProfile
**Description**: Represents user-specific settings that adapt content based on background and preferences

**Fields**:
- id (string, UUID): Unique identifier for the profile
- user_id (string, UUID): Reference to User
- background_level (string): Overall level based on user background (beginner, intermediate, advanced)
- content_depth (string): Desired depth of content (shallow, medium, deep)
- example_preference (string): Type of examples preferred (theoretical, practical, hardware-focused)
- language_preference (string): Preferred language (en, ur)
- personalization_rules (json): Specific rules for content adaptation
- created_at (datetime): Creation timestamp
- updated_at (datetime): Last update timestamp

**Relationships**:
- One User to One PersonalizationProfile
- One PersonalizationProfile to Many UserProgress (via User)

**Validation**:
- Background level must be one of the predefined options
- Content depth must be one of the predefined options
- Language preference must be supported

## Entity: TranslationCache
**Description**: Represents cached translations of content in Urdu to improve performance

**Fields**:
- id (string, UUID): Unique identifier for the cache entry
- content_id (string, UUID): Reference to BookContent
- source_language (string): Original language (en)
- target_language (string): Target language (ur)
- translated_content (text): The translated content
- human_reviewed (boolean): Whether the translation has been reviewed by a human
- review_notes (text): Notes from human reviewer
- created_at (datetime): When the translation was created
- updated_at (datetime): When the translation was last updated
- version (string): Version of the translation

**Relationships**:
- One BookContent to Many TranslationCache (for different languages)
- One TranslationCache to Many UserProgress (when viewed in translated form)

**Validation**:
- Source and target languages must be different
- Both content ID and translated content must not be empty
- Content ID must reference a valid BookContent