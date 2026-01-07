/**
 * Chat Service Hook
 * Provides functions for managing chat sessions, messages, and interactions with the backend API
 * Designed to work with React/Docusaurus applications
 */

import { useState, useEffect, useCallback } from 'react';
import axios from 'axios';

// Configuration
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1';
const CHAT_ENDPOINT = `${API_BASE_URL}/chat`;

// Create axios instance with default configuration
const apiClient = axios.create({
  baseURL: API_BASE_URL,
  timeout: 30000, // 30 seconds timeout for chat operations
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

// Simple in-memory cache for chat sessions
class ChatCache {
  constructor() {
    this.cache = new Map();
    this.maxSize = 50; // Maximum number of items to cache
    this.defaultTTL = 10 * 60 * 1000; // 10 minutes TTL
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

const chatCache = new ChatCache();

/**
 * Hook for chat session management functionality
 * Provides state management and caching for chat operations
 */
export const useChatService = () => {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  /**
   * Create a new chat session
   * @param {Object} sessionData - Session configuration data
   * @param {string} sessionData.selectedText - Optional selected text to focus on
   * @param {string} sessionData.mode - Chat mode ('general' or 'selected-text-only')
   * @returns {Promise<Object>} Created chat session
   */
  const createChatSession = useCallback(async (sessionData = {}) => {
    setLoading(true);
    setError(null);

    try {
      const response = await apiClient.post('/chat/messages', {
        content: 'Hello, I would like to start a chat session.',
        selected_text: sessionData.selectedText || null,
        mode: sessionData.mode || 'general'
      });

      // Cache the session
      chatCache.set(`session_${response.data.session_id}`, response.data);

      setLoading(false);
      return response.data;
    } catch (err) {
      console.error('Error creating chat session:', err);
      setError(err.message || 'Error creating chat session');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Get all chat sessions for the current user
   * @param {Object} params - Pagination parameters
   * @param {number} [params.skip=0] - Number of items to skip
   * @param {number} [params.limit=100] - Maximum number of items to return
   * @returns {Promise<Array>} Array of chat sessions
   */
  const getChatSessions = useCallback(async ({ skip = 0, limit = 100 } = {}) => {
    const cacheKey = `sessions_${skip}_${limit}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = chatCache.get(cacheKey);
    if (cached) {
      console.log('Chat sessions loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      const params = { skip, limit };
      const response = await apiClient.get('/chat/sessions', { params });

      // Cache successful response
      chatCache.set(cacheKey, response.data);

      console.log('Chat sessions loaded from API:', cacheKey);
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error('Error fetching chat sessions:', err);
      setError(err.message || 'Error fetching chat sessions');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Get messages for a specific chat session
   * @param {string} sessionId - Unique identifier of the chat session
   * @param {Object} params - Pagination parameters
   * @param {number} [params.skip=0] - Number of items to skip
   * @param {number} [params.limit=100] - Maximum number of items to return
   * @returns {Promise<Array>} Array of messages in the session
   */
  const getChatSessionMessages = useCallback(async (sessionId, { skip = 0, limit = 100 } = {}) => {
    if (!sessionId) {
      throw new Error('Session ID is required');
    }

    const cacheKey = `messages_${sessionId}_${skip}_${limit}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = chatCache.get(cacheKey);
    if (cached) {
      console.log('Chat messages loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      const params = { skip, limit };
      const response = await apiClient.get(`/chat/sessions/${sessionId}/messages`, { params });

      // Cache successful response
      chatCache.set(cacheKey, response.data);

      console.log('Chat messages loaded from API:', cacheKey);
      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error fetching messages for session ${sessionId}:`, err);
      setError(err.message || 'Error fetching chat messages');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Send a message in a specific chat session
   * @param {string} sessionId - Unique identifier of the chat session
   * @param {string} messageContent - Content of the message to send
   * @returns {Promise<Object>} Response from the AI
   */
  const sendChatMessage = useCallback(async (sessionId, messageContent) => {
    if (!sessionId) {
      throw new Error('Session ID is required');
    }

    if (!messageContent || messageContent.trim().length === 0) {
      throw new Error('Message content is required');
    }

    setLoading(true);
    setError(null);

    try {
      const response = await apiClient.post(`/chat/sessions/${sessionId}/messages`, {
        content: messageContent
      });

      // Clear related cache entries since we have new messages
      chatCache.clear();

      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error sending message to session ${sessionId}:`, err);
      setError(err.message || 'Error sending message');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Close a chat session
   * @param {string} sessionId - Unique identifier of the chat session to close
   * @returns {Promise<boolean>} Success status
   */
  const closeChatSession = useCallback(async (sessionId) => {
    if (!sessionId) {
      throw new Error('Session ID is required');
    }

    setLoading(true);
    setError(null);

    try {
      // Note: The backend doesn't seem to have an explicit close session endpoint
      // We'll just clear the cache for this session
      chatCache.clear();
      
      setLoading(false);
      return true;
    } catch (err) {
      console.error(`Error closing session ${sessionId}:`, err);
      setError(err.message || 'Error closing chat session');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Update chat session properties (like mode or selected text)
   * @param {string} sessionId - Unique identifier of the chat session
   * @param {Object} updates - Properties to update
   * @returns {Promise<Object>} Updated session
   */
  const updateChatSession = useCallback(async (sessionId, updates) => {
    if (!sessionId) {
      throw new Error('Session ID is required');
    }

    setLoading(true);
    setError(null);

    try {
      // For now, we'll create a new session with the updated parameters
      // since there's no explicit update endpoint
      const response = await apiClient.post('/chat/messages', {
        content: 'I want to update my chat session settings.',
        selected_text: updates.selectedText || null,
        mode: updates.mode || 'general'
      });

      // Cache the updated session
      chatCache.set(`session_${response.data.session_id}`, response.data);

      setLoading(false);
      return response.data;
    } catch (err) {
      console.error(`Error updating session ${sessionId}:`, err);
      setError(err.message || 'Error updating chat session');
      setLoading(false);
      throw err;
    }
  }, []);

  /**
   * Get a specific chat session by ID
   * @param {string} sessionId - Unique identifier of the chat session
   * @returns {Promise<Object>} Chat session data
   */
  const getChatSessionById = useCallback(async (sessionId) => {
    if (!sessionId) {
      throw new Error('Session ID is required');
    }

    const cacheKey = `session_${sessionId}`;

    setLoading(true);
    setError(null);

    // Try to get from cache first
    const cached = chatCache.get(cacheKey);
    if (cached) {
      console.log('Chat session loaded from cache:', cacheKey);
      setLoading(false);
      return cached;
    }

    try {
      // Get all sessions and find the one we need
      const sessions = await getChatSessions({ limit: 1000 }); // Get all sessions
      const session = sessions.find(s => s.id === sessionId);

      if (!session) {
        throw new Error('Session not found');
      }

      // Cache successful response
      chatCache.set(cacheKey, session);

      console.log('Chat session loaded from API:', cacheKey);
      setLoading(false);
      return session;
    } catch (err) {
      console.error(`Error fetching session ${sessionId}:`, err);
      setError(err.message || 'Error fetching chat session');
      setLoading(false);
      throw err;
    }
  }, [getChatSessions]);

  /**
   * Clear chat cache
   */
  const clearChatCache = () => {
    chatCache.clear();
    console.log('Chat cache cleared');
  };

  return {
    createChatSession,
    getChatSessions,
    getChatSessionMessages,
    sendChatMessage,
    closeChatSession,
    updateChatSession,
    getChatSessionById,
    clearChatCache,
    loading,
    error
  };
};

export default useChatService;