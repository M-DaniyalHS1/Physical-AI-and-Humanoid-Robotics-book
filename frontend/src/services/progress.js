/**
 * Progress Tracking Service
 * Provides functions for saving, loading, and managing user progress in the textbook platform
 */

import { useState, useCallback } from 'react';
import axios from 'axios';

// Configuration
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1';
const PROGRESS_ENDPOINT = `${API_BASE_URL}/content`;

// Create axios instance with default configuration
const apiClient = axios.create({
  baseURL: API_BASE_URL,
  timeout: 10000, // 10 seconds timeout
  headers: {
    'Content-Type': 'application/json',
  },
});

// Request interceptor to add auth token if available
apiClient.interceptors.request.use(
  (config) => {
    const token = localStorage.getItem('authToken');
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

// Response interceptor to handle common error cases
apiClient.interceptors.response.use(
  (response) => response,
  (error) => {
    if (error.response?.status === 401) {
      // Unauthorized - clear auth token and redirect to login
      localStorage.removeItem('authToken');
      if (typeof window !== 'undefined') {
        window.location.href = '/login';
      }
    }
    return Promise.reject(error);
  }
);

// Simple in-memory cache for progress data
class ProgressCache {
  constructor() {
    this.cache = new Map();
    this.maxSize = 50; // Maximum number of items to cache
    this.defaultTTL = 5 * 60 * 1000; // 5 minutes TTL
  }

  get(key) {
    const item = this.cache.get(key);
    if (!item) return null;

    // Check if item has expired
    if (Date.now() > item.expiry) {
      this.cache.delete(key);
      return null;
    }

    return item.value;
  }

  set(key, value, ttl = this.defaultTTL) {
    // Remove oldest item if cache is at max size
    if (this.cache.size >= this.maxSize) {
      const firstKey = this.cache.keys().next().value;
      this.cache.delete(firstKey);
    }

    this.cache.set(key, {
      value,
      expiry: Date.now() + ttl,
    });
  }

  clear() {
    this.cache.clear();
  }
}

const progressCache = new ProgressCache();

/**
 * Hook for progress tracking functionality
 * Provides state management and caching for progress operations
 */
export const useProgressService = () => {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  /**
   * Get user progress for a specific content item
   * @param {string} contentId - Unique identifier of the content
   * @param {string} userId - Unique identifier of the user
   * @returns {Promise<Object>} User progress data
   */
  const getUserProgress = useCallback(async (contentId, userId) => {
    const cacheKey = `progress_${contentId}_${userId}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = progressCache.get(cacheKey);
    if (cached) {
      console.log('Progress loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      const response = await apiClient.get(`${PROGRESS_ENDPOINT}/${contentId}/progress`, {
        params: { user_id: userId }
      });

      // Cache successful response
      progressCache.set(cacheKey, response.data);

      console.log('Progress loaded from API:', cacheKey);
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error fetching user progress for content ${contentId}, user ${userId}:`, err);
      setError(err.message || 'Error fetching user progress');

      // Return default progress if none exists
      if (err.response?.status === 404) {
        setLoading(false);
        return {
          id: `${userId}_${contentId}`,
          user_id: userId,
          content_id: contentId,
          progress_percentage: 0,
          time_spent_seconds: 0,
          last_accessed: null,
          completed_exercises: [],
          bookmark_position: 0,
          created_at: null,
          updated_at: null
        };
      }

      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Get all user progress records for a specific user
   * @param {string} userId - Unique identifier of the user
   * @returns {Promise<Array>} Array of user progress records
   */
  const getAllUserProgress = useCallback(async (userId) => {
    const cacheKey = `progress_all_${userId}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = progressCache.get(cacheKey);
    if (cached) {
      console.log('All user progress loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      // Since the backend doesn't have a direct /progress endpoint to get all progress,
      // we'll need to get all content first, then get progress for each content item
      // For now, we'll return an empty array to be consistent with the API design
      // In practice, this might require a dedicated backend endpoint
      console.warn('Note: Backend does not have a direct endpoint to fetch all user progress. This function may require backend implementation.');

      // For now, return an empty array since there's no dedicated endpoint
      // In a real implementation, this would call a backend endpoint like /progress?user_id={userId}
      const progressList = [];

      setLoading(false);
      return progressList;
    } catch (err) {
      console.error(`Error fetching all user progress for user ${userId}:`, err);
      setError(err.message || 'Error fetching user progress');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Update user progress for specific content
   * @param {string} contentId - Unique identifier of the content
   * @param {string} userId - Unique identifier of the user
   * @param {number} progressPercentage - Progress percentage (0-100)
   * @param {Object} progressData - Additional progress data
   * @returns {Promise<boolean>} Success status
   */
  const updateUserProgress = useCallback(async (contentId, userId, progressPercentage, progressData = {}) => {
    setLoading(true);
    setError(null);

    try {
      const response = await apiClient.post(`${PROGRESS_ENDPOINT}/${contentId}/progress`, {
        user_id: userId,
        progress_percentage: progressPercentage,
        time_spent_seconds: progressData.timeSpentSeconds,
        completed_exercises: progressData.completedExercises,
        bookmark_position: progressData.bookmarkPosition
      }, {
        headers: {
          'Content-Type': 'application/json',
        }
      });

      // Clear related cache entries
      progressCache.clear(); // For simplicity, clear entire cache on updates

      console.log('User progress updated successfully');
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error updating user progress for content ${contentId}, user ${userId}:`, err);
      setError(err.message || 'Error updating user progress');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Save user progress with bookmark position
   * @param {string} contentId - Unique identifier of the content
   * @param {string} userId - Unique identifier of the user
   * @param {number} progressPercentage - Progress percentage (0-100)
   * @param {number} bookmarkPosition - Position in the content to bookmark
   * @returns {Promise<boolean>} Success status
   */
  const saveBookmarkedProgress = useCallback(async (contentId, userId, progressPercentage, bookmarkPosition) => {
    setLoading(true);
    setError(null);

    try {
      const response = await apiClient.post(`${PROGRESS_ENDPOINT}/${contentId}/progress`, {
        user_id: userId,
        progress_percentage: progressPercentage,
        bookmark_position: bookmarkPosition
      }, {
        headers: {
          'Content-Type': 'application/json',
        }
      });

      // Clear related cache entries
      progressCache.clear();

      console.log('Bookmarked progress saved successfully');
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error saving bookmarked progress for content ${contentId}, user ${userId}:`, err);
      setError(err.message || 'Error saving bookmarked progress');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Log time spent on content
   * @param {string} contentId - Unique identifier of the content
   * @param {string} userId - Unique identifier of the user
   * @param {number} timeSpentSeconds - Time spent in seconds
   * @returns {Promise<boolean>} Success status
   */
  const logTimeSpent = useCallback(async (contentId, userId, timeSpentSeconds) => {
    setLoading(true);
    setError(null);

    try {
      // First get current progress
      const currentProgress = await getUserProgress(contentId, userId);
      
      // Calculate new time spent
      const newTimeSpent = (currentProgress.time_spent_seconds || 0) + timeSpentSeconds;
      
      // Update progress with new time
      const response = await apiClient.post(`${PROGRESS_ENDPOINT}/${contentId}/progress`, {
        user_id: userId,
        progress_percentage: currentProgress.progress_percentage || 0,
        time_spent_seconds: newTimeSpent
      }, {
        headers: {
          'Content-Type': 'application/json',
        }
      });

      // Clear related cache entries
      progressCache.clear();

      console.log('Time spent logged successfully');
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error logging time spent on content ${contentId}, user ${userId}:`, err);
      setError(err.message || 'Error logging time spent');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Mark exercise as completed
   * @param {string} contentId - Unique identifier of the content
   * @param {string} userId - Unique identifier of the user
   * @param {string} exerciseId - Unique identifier of the exercise
   * @returns {Promise<boolean>} Success status
   */
  const markExerciseCompleted = useCallback(async (contentId, userId, exerciseId) => {
    setLoading(true);
    setError(null);

    try {
      // First get current progress
      const currentProgress = await getUserProgress(contentId, userId);
      
      // Add exercise to completed list if not already there
      const completedExercises = currentProgress.completed_exercises || [];
      if (!completedExercises.includes(exerciseId)) {
        completedExercises.push(exerciseId);
      }
      
      // Update progress with completed exercise
      const response = await apiClient.post(`${PROGRESS_ENDPOINT}/${contentId}/progress`, {
        user_id: userId,
        progress_percentage: currentProgress.progress_percentage || 0,
        completed_exercises: completedExercises
      }, {
        headers: {
          'Content-Type': 'application/json',
        }
      });

      // Clear related cache entries
      progressCache.clear();

      console.log('Exercise marked as completed successfully');
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error marking exercise as completed for content ${contentId}, user ${userId}, exercise ${exerciseId}:`, err);
      setError(err.message || 'Error marking exercise as completed');
      setLoading(false);
      throw err;
    }
  }, []);

  // Export the cache for manual management if needed
  const clearProgressCache = () => {
    progressCache.clear();
    console.log('Progress cache cleared');
  };

  return {
    getUserProgress,
    getAllUserProgress,
    updateUserProgress,
    saveBookmarkedProgress,
    logTimeSpent,
    markExerciseCompleted,
    clearProgressCache,
    loading,
    error
  };
};

export default useProgressService;