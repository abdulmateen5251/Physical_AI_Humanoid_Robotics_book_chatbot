import React, { ReactNode } from 'react';
import { useAuth } from './AuthContext';
import './ProtectedContent.css';

interface ProtectedContentProps {
  children: ReactNode;
  onLoginRequired: () => void;
  showLoginButton?: boolean;
}

const ProtectedContent: React.FC<ProtectedContentProps> = ({
  children,
  onLoginRequired,
  showLoginButton = true,
}) => {
  const { isAuthenticated, isLoading, user } = useAuth();

  if (isLoading) {
    return (
      <div className="protected-loading">
        <div className="protected-spinner"></div>
        <p>Loading...</p>
      </div>
    );
  }

  if (!isAuthenticated) {
    return (
      <div className="protected-message">
        <div className="protected-icon">🔒</div>
        <h3>Authentication Required</h3>
        <p>Please sign in to access the chatbot and ask questions about the course content.</p>
        {showLoginButton && (
          <button className="protected-login-button" onClick={onLoginRequired}>
            Sign In to Continue
          </button>
        )}
      </div>
    );
  }

  return <>{children}</>;
};

export default ProtectedContent;
