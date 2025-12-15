// Frontend authentication service using Better Auth
// This will handle the client-side authentication with Better Auth

import { createAuthClient } from "better-auth/client";

// Initialize Better Auth client
export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000',
});

// Signup function - collects user background information
export const signup = async (userData) => {
  const { email, password, softwareExperience, hardwareExperience, mathPhysicsLevel, learningGoals } = userData;

  try {
    // Use Better Auth's built-in signup function
    const response = await authClient.signUp.email({
      email,
      password,
      // Add custom fields for user background
      $fields: {
        email,
        password,
        software_experience: softwareExperience,
        hardware_experience: hardwareExperience,
        math_physics_level: mathPhysicsLevel,
        learning_goals: learningGoals
      }
    });

    return response;
  } catch (error) {
    console.error('Signup error:', error);
    throw error;
  }
};

// Login function
export const login = async (email, password) => {
  try {
    // Use Better Auth's built-in login function
    const response = await authClient.signIn.email({
      email,
      password
    });

    return response;
  } catch (error) {
    console.error('Login error:', error);
    throw error;
  }
};

// Logout function
export const logout = async () => {
  try {
    await authClient.signOut();
  } catch (error) {
    console.error('Logout error:', error);
    throw error;
  }
};

// Get current user function
export const getCurrentUser = async () => {
  try {
    const session = await authClient.getSession();
    return session?.user;
  } catch (error) {
    console.error('Get user error:', error);
    throw error;
  }
};

// Check if user is authenticated
export const isAuthenticated = async () => {
  try {
    const session = await authClient.getSession();
    return !!session;
  } catch (error) {
    console.error('Auth check error:', error);
    return false;
  }
};