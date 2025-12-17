/**
 * Content Service Hook
 * Provides functions for loading, caching, and managing textbook content from the backend API
 * Designed to work with React/Docusaurus applications
 */

import { useState, useEffect, useCallback } from 'react';
import axios from 'axios';

// Configuration
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1';
const CONTENT_ENDPOINT = `${API_BASE_URL}/content`;
const CONTENT_METADATA_ENDPOINT = `${API_BASE_URL}/content`;

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

// Simple in-memory cache for content
class ContentCache {
  constructor() {
    this.cache = new Map();
    this.maxSize = 100; // Maximum number of items to cache
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

const contentCache = new ContentCache();

/**
 * Hook for content loading functionality
 * Provides state management and caching for content operations
 */
export const useContentService = () => {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  /**
   * Fetch all content with pagination and optional filtering
   * @param {Object} params - Filter and pagination parameters
   * @param {number} [params.skip=0] - Number of items to skip
   * @param {number} [params.limit=100] - Maximum number of items to return
   * @param {string} [params.module] - Filter by module
   * @param {string} [params.contentType] - Filter by content type
   * @returns {Promise<Array>} Array of content items
   */
  const getAllContent = useCallback(async ({ skip = 0, limit = 100, module, contentType } = {}) => {
    const cacheKey = `content_${skip}_${limit}_${module || 'all'}_${contentType || 'all'}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = contentCache.get(cacheKey);
    if (cached) {
      console.log('Content loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      const params = {
        skip,
        limit,
        ...(module && { module }),
        ...(contentType && { content_type: contentType }),
      };

      const response = await apiClient.get(CONTENT_ENDPOINT, { params });

      // Cache successful response
      contentCache.set(cacheKey, response.data);

      console.log('Content loaded from API:', cacheKey);
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error('Error fetching all content:', err);
      setError(err.message || 'Error fetching content');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Fetch specific content by ID
   * @param {string} contentId - Unique identifier of the content
   * @returns {Promise<Object>} Content item
   */
  const getContentById = useCallback(async (contentId) => {
    const cacheKey = `content_${contentId}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = contentCache.get(cacheKey);
    if (cached) {
      console.log('Content loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      const response = await apiClient.get(`${CONTENT_ENDPOINT}/${contentId}`);

      // Cache successful response
      contentCache.set(cacheKey, response.data);

      console.log('Content loaded from API:', cacheKey);
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error fetching content by ID ${contentId}:`, err);
      setError(err.message || 'Error fetching content');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Fetch content metadata for personalization
   * @param {string} contentId - Unique identifier of the content
   * @param {string} [level] - Personalization level (beginner, intermediate, advanced)
   * @returns {Promise<Object>} Content metadata
   */
  const getContentMetadata = useCallback(async (contentId, level) => {
    const cacheKey = `metadata_${contentId}_${level || 'all'}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = contentCache.get(cacheKey);
    if (cached) {
      console.log('Content metadata loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      const params = {};
      if (level) {
        params.level = level;
      }

      const response = await apiClient.get(`${CONTENT_METADATA_ENDPOINT}/${contentId}/metadata`, { params });

      // Cache successful response
      contentCache.set(cacheKey, response.data);

      console.log('Content metadata loaded from API:', cacheKey);
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error fetching content metadata for ${contentId}:`, err);
      setError(err.message || 'Error fetching content metadata');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Fetch user progress for specific content
   * @param {string} contentId - Unique identifier of the content
   * @param {string} userId - Unique identifier of the user
   * @returns {Promise<Object>} User progress data
   */
  const getUserProgress = useCallback(async (contentId, userId) => {
    setLoading(true);
    setError(null);

    try {
      const response = await apiClient.get(`${CONTENT_METADATA_ENDPOINT}/${contentId}/progress`, {
        params: { user_id: userId }
      });

      console.log('User progress loaded from API');
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
      const response = await apiClient.post(`${CONTENT_METADATA_ENDPOINT}/${contentId}/progress`, {
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
      contentCache.clear(); // For simplicity, clear entire cache on updates

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
   * Search content by keyword
   * @param {string} query - Search query
   * @param {string} [contentId='*'] - Content ID to search in, '*' for all content
   * @returns {Promise<Array>} Search results
   */
  const searchContent = useCallback(async (query, contentId = '*') => {
    if (!query || query.trim().length < 2) {
      return [];
    }

    const trimmedQuery = query.trim().toLowerCase();
    const cacheKey = `search_${contentId}_${trimmedQuery}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = contentCache.get(cacheKey);
    if (cached) {
      console.log('Search results loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      const response = await apiClient.get(`${CONTENT_METADATA_ENDPOINT}/${contentId}/search`, {
        params: { query: trimmedQuery }
      });

      // Cache successful response (shorter TTL for search results)
      contentCache.set(cacheKey, response.data, 60 * 1000); // 1 minute for search results

      console.log('Search results loaded from API:', cacheKey);
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error searching content with query "${query}":`, err);
      setError(err.message || 'Error searching content');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Fetch content by module
   * @param {string} module - Module name (e.g., 'ROS 2', 'Gazebo & Unity', etc.)
   * @param {Object} params - Pagination parameters
   * @returns {Promise<Array>} Array of content items in the specified module
   */
  const getContentByModule = useCallback(async (module, { skip = 0, limit = 100 } = {}) => {
    if (!module) {
      throw new Error('Module parameter is required');
    }

    const cacheKey = `module_${module}_${skip}_${limit}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = contentCache.get(cacheKey);
    if (cached) {
      console.log('Module content loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      const response = await apiClient.get(CONTENT_ENDPOINT, {
        params: { module, skip, limit }
      });

      // Cache successful response
      contentCache.set(cacheKey, response.data);

      console.log('Module content loaded from API:', cacheKey);
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error fetching content for module ${module}:`, err);
      setError(err.message || 'Error fetching content');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Fetch content by type
   * @param {string} contentType - Content type (e.g., 'text', 'video', 'diagram', etc.)
   * @param {Object} params - Pagination parameters
   * @returns {Promise<Array>} Array of content items of the specified type
   */
  const getContentByType = useCallback(async (contentType, { skip = 0, limit = 100 } = {}) => {
    if (!contentType) {
      throw new Error('ContentType parameter is required');
    }

    const cacheKey = `type_${contentType}_${skip}_${limit}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = contentCache.get(cacheKey);
    if (cached) {
      console.log('Type-based content loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      const response = await apiClient.get(CONTENT_ENDPOINT, {
        params: { content_type: contentType, skip, limit }
      });

      // Cache successful response
      contentCache.set(cacheKey, response.data);

      console.log('Type-based content loaded from API:', cacheKey);
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error fetching content of type ${contentType}:`, err);
      setError(err.message || 'Error fetching content');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Get content recommendations for user based on their progress and preferences
   * @param {string} userId - Unique identifier of the user
   * @param {number} [limit=10] - Maximum number of recommendations to return
   * @returns {Promise<Array>} Recommended content items
   */
  const getContentRecommendations = useCallback(async (userId, limit = 10) => {
    if (!userId) {
      throw new Error('UserId parameter is required');
    }

    const cacheKey = `recommendations_${userId}_${limit}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = contentCache.get(cacheKey);
    if (cached) {
      console.log('Recommendations loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      // This would normally involve more complex logic in a real system
      // For now, we'll fetch popular content based on some heuristic
      const response = await getAllContent({ skip: 0, limit });

      // Cache successful response
      contentCache.set(cacheKey, response);

      console.log('Recommendations loaded from API:', cacheKey);
      setLoading(false);
      return response;
    } catch (err) {
      console.error(`Error fetching recommendations for user ${userId}:`, err);
      setError(err.message || 'Error fetching recommendations');
      setLoading(false);
      throw err;
    }
  }, [getAllContent]);

  // Export the cache for manual management if needed
  const clearContentCache = () => {
    contentCache.clear();
    console.log('Content cache cleared');
  };

  return {
    getAllContent,
    getContentById,
    getContentMetadata,
    getUserProgress,
    updateUserProgress,
    searchContent,
    getContentByModule,
    getContentByType,
    getContentRecommendations,
    clearContentCache,
    loading,
    error
  };
};

export default useContentService;