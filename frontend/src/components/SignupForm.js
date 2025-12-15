import React, { useState } from 'react';
import { signup } from '../services/auth';

const SignupForm = ({ onComplete }) => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    softwareExperience: 'beginner',
    hardwareExperience: 'beginner',
    mathPhysicsLevel: 'beginner',
    learningGoals: ''
  });
  const [error, setError] = useState('');

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    
    try {
      // Call the signup function with the collected data
      const result = await signup(formData);
      console.log('Signup successful:', result);
      
      // Optionally call onComplete to handle post-signup actions
      if (onComplete) {
        onComplete(result);
      }
    } catch (err) {
      setError(err.message || 'An error occurred during signup');
      console.error('Signup error:', err);
    }
  };

  return (
    <div className="signup-form">
      <h2>Sign Up for AI Textbook Platform</h2>
      {error && <div className="error-message">{error}</div>}
      
      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="email">Email:</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
          />
        </div>
        
        <div className="form-group">
          <label htmlFor="password">Password:</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
          />
        </div>
        
        <div className="form-group">
          <label htmlFor="softwareExperience">Software Experience:</label>
          <select
            id="softwareExperience"
            name="softwareExperience"
            value={formData.softwareExperience}
            onChange={handleChange}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>
        
        <div className="form-group">
          <label htmlFor="hardwareExperience">Hardware Experience:</label>
          <select
            id="hardwareExperience"
            name="hardwareExperience"
            value={formData.hardwareExperience}
            onChange={handleChange}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>
        
        <div className="form-group">
          <label htmlFor="mathPhysicsLevel">Math/Physics Level:</label>
          <select
            id="mathPhysicsLevel"
            name="mathPhysicsLevel"
            value={formData.mathPhysicsLevel}
            onChange={handleChange}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>
        
        <div className="form-group">
          <label htmlFor="learningGoals">Learning Goals:</label>
          <textarea
            id="learningGoals"
            name="learningGoals"
            value={formData.learningGoals}
            onChange={handleChange}
            placeholder="Describe your learning goals and objectives"
          />
        </div>
        
        <button type="submit">Sign Up</button>
      </form>
    </div>
  );
};

export default SignupForm;