import React, { useState } from 'react';
import './AuthModal.css';
import { useAuth } from './AuthContext';

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSuccess: () => void;
}

const AuthModal: React.FC<AuthModalProps> = ({ isOpen, onClose, onSuccess }) => {
  const [isSignup, setIsSignup] = useState(false);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const { login, signup } = useAuth();

  if (!isOpen) return null;

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      if (isSignup) {
        await signup(email, password);
      } else {
        await login(email, password);
      }

      // Success
      setEmail('');
      setPassword('');
      onSuccess();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Authentication failed');
    } finally {
      setLoading(false);
    }
  };

  const handleOverlayClick = (e: React.MouseEvent) => {
    if (e.target === e.currentTarget) {
      onClose();
    }
  };

  return (
    <div className="auth-modal-overlay" onClick={handleOverlayClick}>
      <div className="auth-modal">
        <div className="auth-modal-header">
          <h2>{isSignup ? 'Create Account' : 'Sign In'}</h2>
          <button className="auth-modal-close" onClick={onClose}>
            ×
          </button>
        </div>

        <form onSubmit={handleSubmit} className="auth-form">
          {error && (
            <div className="auth-error">
              {error}
            </div>
          )}

          <div className="auth-form-group">
            <label htmlFor="email">Email</label>
            <input
              id="email"
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              placeholder="your@email.com"
              required
              autoComplete="email"
            />
          </div>

          <div className="auth-form-group">
            <label htmlFor="password">Password</label>
            <div className="auth-password-wrapper">
              <input
                id="password"
                type={showPassword ? 'text' : 'password'}
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="••••••••"
                required
                minLength={8}
                autoComplete={isSignup ? 'new-password' : 'current-password'}
              />
              <button
                type="button"
                className="auth-password-toggle"
                onClick={() => setShowPassword(!showPassword)}
                aria-label={showPassword ? 'Hide password' : 'Show password'}
              >
                {showPassword ? '👁️' : '👁️‍🗨️'}
              </button>
            </div>
            {isSignup && (
              <small className="auth-hint">Minimum 8 characters</small>
            )}
          </div>

          <button
            type="submit"
            className="auth-submit-button"
            disabled={loading}
          >
            {loading ? 'Please wait...' : isSignup ? 'Sign Up' : 'Sign In'}
          </button>

          <div className="auth-toggle">
            {isSignup ? (
              <>
                Already have an account?{' '}
                <button
                  type="button"
                  className="auth-toggle-link"
                  onClick={() => {
                    setIsSignup(false);
                    setError('');
                  }}
                >
                  Sign In
                </button>
              </>
            ) : (
              <>
                Don't have an account?{' '}
                <button
                  type="button"
                  className="auth-toggle-link"
                  onClick={() => {
                    setIsSignup(true);
                    setError('');
                  }}
                >
                  Sign Up
                </button>
              </>
            )}
          </div>
        </form>
      </div>
    </div>
  );
};

export default AuthModal;
