/**
 * Content Display Component
 * Displays textbook content with personalized formatting based on user preferences
 */

import React, { useState, useEffect, useCallback } from 'react';
import { useContentService } from '../services/content';

const ContentDisplay = ({ contentId, userId, personalizationLevel }) => {
  const { 
    getContentById, 
    getContentMetadata, 
    getUserProgress, 
    updateUserProgress,
    searchContent,
    loading,
    error 
  } = useContentService();
  
  const [content, setContent] = useState(null);
  const [contentMetadata, setContentMetadata] = useState(null);
  const [userProgress, setUserProgress] = useState({
    progress_percentage: 0,
    time_spent_seconds: 0,
    completed_exercises: [],
    bookmark_position: 0
  });
  const [searchQuery, setSearchQuery] = useState('');
  const [searchResults, setSearchResults] = useState([]);
  const [showSearchResults, setShowSearchResults] = useState(false);
  const [lastReadPosition, setLastReadPosition] = useState(0);
  const [isLoading, setIsLoading] = useState(false);
  const [displayError, setDisplayError] = useState(null);

  // Load content when component mounts or contentId changes
  useEffect(() => {
    if (contentId) {
      loadContent();
    }
  }, [contentId]);

  // Load user progress when content or user changes
  useEffect(() => {
    if (contentId && userId) {
      loadUserProgress();
    }
  }, [contentId, userId]);

  // Load content and related metadata
  const loadContent = async () => {
    setIsLoading(true);
    setDisplayError(null);

    try {
      // Load primary content
      const loadedContent = await getContentById(contentId);
      setContent(loadedContent);

      // Load content metadata based on personalization level
      if (personalizationLevel) {
        const metadata = await getContentMetadata(contentId, personalizationLevel);
        setContentMetadata(metadata);
      } else {
        // Load default metadata
        const metadata = await getContentMetadata(contentId);
        setContentMetadata(metadata);
      }

      // Restore reading position from user progress
      if (userProgress && userProgress.bookmark_position) {
        setLastReadPosition(userProgress.bookmark_position);
      }
    } catch (err) {
      setDisplayError(err.message || 'Failed to load content');
      console.error('Error loading content:', err);
    } finally {
      setIsLoading(false);
    }
  };

  // Load user progress
  const loadUserProgress = async () => {
    try {
      const progress = await getUserProgress(contentId, userId);
      setUserProgress(progress);
      if (progress.bookmark_position) {
        setLastReadPosition(progress.bookmark_position);
      }
    } catch (err) {
      console.error('Error loading user progress:', err);
      // Use default progress state
    }
  };

  // Update user progress as they read
  const updateReadingProgress = useCallback(async (scrollPosition) => {
    if (!userId || !contentId) return;

    // Calculate progress percentage based on scroll position
    const totalHeight = document.documentElement.scrollHeight - window.innerHeight;
    const currentProgress = totalHeight > 0 ? Math.min(100, Math.floor((scrollPosition / totalHeight) * 100)) : 0;

    // Only update if progress has changed significantly (e.g., 5% increments)
    if (Math.abs(currentProgress - userProgress.progress_percentage) >= 5) {
      try {
        const updatedProgress = {
          ...userProgress,
          progress_percentage: currentProgress,
          bookmark_position: Math.floor(scrollPosition)
        };

        await updateUserProgress(
          contentId,
          userId,
          currentProgress,
          {
            timeSpentSeconds: userProgress.time_spent_seconds,
            completedExercises: userProgress.completed_exercises,
            bookmarkPosition: Math.floor(scrollPosition)
          }
        );

        setUserProgress(updatedProgress);
      } catch (err) {
        console.error('Error updating user progress:', err);
      }
    }
  }, [userId, contentId, userProgress]);

  // Handle scroll events to track reading progress
  useEffect(() => {
    if (!contentId || !userId) return;

    const handleScroll = () => {
      updateReadingProgress(window.scrollY);
    };

    // Throttle scroll events to avoid excessive API calls
    let ticking = false;
    const throttledScrollHandler = () => {
      if (!ticking) {
        requestAnimationFrame(() => {
          handleScroll();
          ticking = false;
        });
        ticking = true;
      }
    };

    window.addEventListener('scroll', throttledScrollHandler);

    return () => {
      window.removeEventListener('scroll', throttledScrollHandler);
    };
  }, [updateReadingProgress, contentId, userId]);

  // Handle search
  const handleSearch = async (query) => {
    if (!query || query.trim().length < 2) {
      setSearchResults([]);
      setShowSearchResults(false);
      return;
    }

    try {
      const results = await searchContent(query, contentId);
      setSearchResults(results);
      setShowSearchResults(true);
    } catch (err) {
      console.error('Error searching content:', err);
      setSearchResults([]);
    }
  };

  // Navigate to search result
  const handleSearchResultClick = (result) => {
    // Scroll to the relevant part of the content
    const element = document.getElementById(result.id || 'top');
    if (element) {
      element.scrollIntoView({ behavior: 'smooth' });
    }
    setShowSearchResults(false);
    setSearchQuery('');
  };

  // Format content based on personalization level
  const formatContent = (rawContent, metadata) => {
    if (!rawContent) return '';

    // If we have personalized content in metadata, use that
    if (metadata && metadata.adjusted_content) {
      return metadata.adjusted_content;
    }

    // Otherwise, apply formatting based on the personalization level
    if (personalizationLevel === 'beginner') {
      // For beginners, simplify complex concepts and add more explanations
      return rawContent.replace(/\*\*(.*?)\*\*/g, '<strong className="beginner-highlight">$1</strong>');
    } else if (personalizationLevel === 'advanced') {
      // For advanced users, add more technical depth
      return rawContent;
    }

    // Default formatting
    return rawContent;
  };

  // Render loading state
  if (loading || isLoading) {
    return (
      <div className="content-display loading">
        <div className="loading-spinner">Loading content...</div>
      </div>
    );
  }

  // Render error state
  if (error || displayError) {
    return (
      <div className="content-display error">
        <h2>Error Loading Content</h2>
        <p>{error || displayError}</p>
        <button onClick={loadContent}>Retry</button>
      </div>
    );
  }

  // Render content
  return (
    <div className="content-display" id="top">
      {/* Search Bar */}
      <div className="search-container">
        <input
          type="text"
          value={searchQuery}
          onChange={(e) => {
            setSearchQuery(e.target.value);
            handleSearch(e.target.value);
          }}
          placeholder="Search within this content..."
          className="search-input"
        />
        {showSearchResults && (
          <div className="search-results">
            {searchResults.length > 0 ? (
              <ul>
                {searchResults.map((result, index) => (
                  <li key={index} onClick={() => handleSearchResultClick(result)}>
                    {result.title || result.section || result.substring(0, 100) + '...'}
                  </li>
                ))}
              </ul>
            ) : (
              <p>No results found</p>
            )}
          </div>
        )}
      </div>

      {/* Progress Indicator */}
      <div className="progress-indicator">
        <div className="progress-bar">
          <div 
            className="progress-fill" 
            style={{ width: `${userProgress.progress_percentage}%` }}
          ></div>
        </div>
        <span className="progress-text">{userProgress.progress_percentage}% complete</span>
      </div>

      {/* Content Header */}
      {content && (
        <header className="content-header">
          <h1>{content.title || 'Content Title'}</h1>
          <div className="content-meta">
            <span className="module-tag">{content.module}</span>
            <span className="chapter-number">Chapter {content.chapter_number}</span>
            <span className="content-type">{content.content_type}</span>
          </div>
        </header>
      )}

      {/* Main Content */}
      <main className="content-body">
        {content && (
          <div 
            className="content-text"
            dangerouslySetInnerHTML={{
              __html: formatContent(content.content, contentMetadata)
            }}
          />
        )}

        {/* Learning Objectives */}
        {content && content.learning_objectives && content.learning_objectives.length > 0 && (
          <section className="learning-objectives">
            <h3>Learning Objectives</h3>
            <ul>
              {content.learning_objectives.map((objective, index) => (
                <li key={index}>{objective}</li>
              ))}
            </ul>
          </section>
        )}

        {/* Adjusted Content for Personalization */}
        {contentMetadata && contentMetadata.adjusted_content && (
          <section className="personalized-content">
            <h3>Personalized for {personalizationLevel || 'your level'}</h3>
            <div 
              dangerouslySetInnerHTML={{
                __html: formatContent(contentMetadata.adjusted_content, contentMetadata)
              }}
            />
          </section>
        )}

        {/* Examples for Personalization Level */}
        {contentMetadata && contentMetadata.adjusted_examples && contentMetadata.adjusted_examples.length > 0 && (
          <section className="personalized-examples">
            <h3>Examples</h3>
            <ul>
              {contentMetadata.adjusted_examples.map((example, index) => (
                <li key={index}>{example}</li>
              ))}
            </ul>
          </section>
        )}

        {/* Difficulty Rating */}
        {contentMetadata && contentMetadata.difficulty_rating && (
          <div className="difficulty-rating">
            <label>Difficulty:</label>
            <span>{'★'.repeat(contentMetadata.difficulty_rating)}{'☆'.repeat(5 - contentMetadata.difficulty_rating)}</span>
          </div>
        )}
      </main>

      {/* Exercises/Activities Section */}
      <section className="exercises-section">
        <h3>Knowledge Check</h3>
        <div className="exercises-list">
          {/* This would be populated with exercises based on the content */}
          <p>Exercises would appear here based on the content type and personalization level.</p>
        </div>
      </section>

      {/* Navigation Controls */}
      <footer className="content-navigation">
        <button className="nav-button prev-button">Previous Section</button>
        <button className="nav-button next-button">Next Section</button>
      </footer>
    </div>
  );
};

export default ContentDisplay;