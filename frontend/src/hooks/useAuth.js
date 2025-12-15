import React, { createContext, useContext, useReducer, useEffect } from 'react';
import { isAuthenticated, getCurrentUser, login as loginService, logout as logoutService } from '../services/auth';

const AuthContext = createContext();

const authReducer = (state, action) => {
  switch (action.type) {
    case 'LOGIN_START':
      return { ...state, loading: true, error: null };
    case 'LOGIN_SUCCESS':
      return { 
        ...state, 
        loading: false, 
        isAuthenticated: true, 
        user: action.payload.user,
        token: action.payload.token
      };
    case 'LOGIN_FAILURE':
      return { ...state, loading: false, error: action.payload };
    case 'LOGOUT':
      return { 
        ...state, 
        isAuthenticated: false, 
        user: null, 
        token: null 
      };
    case 'SET_USER':
      return { 
        ...state, 
        user: action.payload 
      };
    case 'CHECK_AUTH_STATUS':
      return { 
        ...state, 
        isAuthenticated: action.payload.isAuthenticated,
        user: action.payload.user
      };
    default:
      return state;
  }
};

const AuthProvider = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, {
    isAuthenticated: false,
    user: null,
    loading: false,
    error: null,
    token: null
  });

  // Check authentication status on initial load
  useEffect(() => {
    const checkAuthStatus = async () => {
      try {
        if (isAuthenticated()) {
          const user = await getCurrentUser();
          dispatch({
            type: 'CHECK_AUTH_STATUS',
            payload: { isAuthenticated: true, user }
          });
        }
      } catch (error) {
        console.error('Auth status check failed:', error);
        // If token is invalid, clear it
        localStorage.removeItem('authToken');
        dispatch({
          type: 'CHECK_AUTH_STATUS',
          payload: { isAuthenticated: false, user: null }
        });
      }
    };

    checkAuthStatus();
  }, []);

  const login = async (email, password) => {
    dispatch({ type: 'LOGIN_START' });
    
    try {
      const response = await loginService(email, password);
      dispatch({
        type: 'LOGIN_SUCCESS',
        payload: {
          user: response.user,
          token: response.access_token
        }
      });
      return response;
    } catch (error) {
      dispatch({
        type: 'LOGIN_FAILURE',
        payload: error.message
      });
      throw error;
    }
  };

  const logout = async () => {
    try {
      await logoutService();
      dispatch({ type: 'LOGOUT' });
    } catch (error) {
      console.error('Logout error:', error);
      // Even if the service fails, we should clear local state
      dispatch({ type: 'LOGOUT' });
    }
  };

  const value = {
    ...state,
    login,
    logout
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export { AuthProvider, useAuth };