import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import AuthModal from './AuthModal';
import './AuthButton.css';

const AuthButton: React.FC = () => {
  const { isAuthenticated, user, logout, isLoading } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [showUserMenu, setShowUserMenu] = useState(false);

  if (isLoading) {
    return null;
  }

  const handleLogout = async () => {
    await logout();
    setShowUserMenu(false);
  };

  if (isAuthenticated && user) {
    return (
      <div className="auth-button-container">
        <button
          className="auth-user-button"
          onClick={() => setShowUserMenu(!showUserMenu)}
        >
          <span className="auth-user-avatar">
            {user.email.charAt(0).toUpperCase()}
          </span>
          <span className="auth-user-email">{user.email}</span>
          <span className="auth-dropdown-arrow">▼</span>
        </button>

        {showUserMenu && (
          <div className="auth-user-menu">
            <div className="auth-user-menu-header">
              <div className="auth-user-menu-email">{user.email}</div>
            </div>
            <button className="auth-user-menu-item" onClick={handleLogout}>
              <span>🚪</span> Sign Out
            </button>
          </div>
        )}
      </div>
    );
  }

  return (
    <>
      <button className="auth-signin-button" onClick={() => setShowAuthModal(true)}>
        Sign In
      </button>
      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onSuccess={() => setShowAuthModal(false)}
      />
    </>
  );
};

export default AuthButton;
