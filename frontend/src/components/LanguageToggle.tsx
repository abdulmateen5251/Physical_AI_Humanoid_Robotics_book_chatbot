import React, { useState, useEffect } from 'react';
import {
  getPreferredLanguage,
  setPreferredLanguage,
  translateElement,
  restoreOriginalContent,
  storeOriginalContent,
  cancelTranslation,
  isTranslationInProgress,
  type Language,
} from '../utils/translation';
import AuthModal from './AuthModal';
import { useAuth } from './AuthContext';
import './LanguageToggle.css';

export default function LanguageToggle() {
  const [language, setLanguage] = useState<Language>('en');
  const [isTranslating, setIsTranslating] = useState(false);
  const [showAuthModal, setShowAuthModal] = useState(false);
  const { isAuthenticated, isLoading } = useAuth();

  useEffect(() => {
    const onStart = () => setIsTranslating(true);
    const onEnd = () => setIsTranslating(false);
    const onCanceled = () => setIsTranslating(false);
    const onProgress = (e: Event) => {
      // Optionally, we could show progress in future
    };

    window.addEventListener('translationStart', onStart as EventListener);
    window.addEventListener('translationEnd', onEnd as EventListener);
    window.addEventListener('translationCanceled', onCanceled as EventListener);
    window.addEventListener('translationProgress', onProgress as EventListener);

    // Load saved preference
    const savedLang = getPreferredLanguage();
    setLanguage(savedLang);

    return () => {
      window.removeEventListener('translationStart', onStart as EventListener);
      window.removeEventListener('translationEnd', onEnd as EventListener);
      window.removeEventListener('translationCanceled', onCanceled as EventListener);
      window.removeEventListener('translationProgress', onProgress as EventListener);
    };
  }, []);

  useEffect(() => {
    // Listen for route changes
    const handleRouteChange = () => {
      if (language === 'ur' && isAuthenticated) {
        setTimeout(() => {
          applyTranslation();
        }, 500); // Delay to ensure content is loaded
      }
    };

    window.addEventListener('popstate', handleRouteChange);
    return () => window.removeEventListener('popstate', handleRouteChange);
  }, [language, isAuthenticated]);

  useEffect(() => {
    if (isLoading) return;

    // If user is not authenticated, keep toggle in English and clear Urdu preference
    if (!isAuthenticated) {
      if (language === 'ur') {
        restoreEnglish();
        setLanguage('en');
        setPreferredLanguage('en');
      }
      return;
    }

    // Auto-apply saved Urdu preference after auth
    if (language === 'ur') {
      applyTranslation();
    }
  }, [isAuthenticated, isLoading, language]);

  const applyTranslation = async () => {
    if (!isAuthenticated) {
      setShowAuthModal(true);
      return;
    }

    setIsTranslating(true);
    
    try {
      // Find the main content area
      const mainContent = document.querySelector('main.docMainContainer_node_modules-\\@docusaurus-theme-classic-lib-theme-DocPage-Layout-Main-styles-module') 
        || document.querySelector('main')
        || document.querySelector('.markdown');
      
      if (mainContent) {
        storeOriginalContent(mainContent as HTMLElement);
        await translateElement(mainContent as HTMLElement);
      }

      // Translate sidebar
      const sidebar = document.querySelector('aside.theme-doc-sidebar-container');
      if (sidebar) {
        storeOriginalContent(sidebar as HTMLElement);
        await translateElement(sidebar as HTMLElement);
      }

      // Translate navbar items (except buttons)
      const navbarItems = document.querySelectorAll('.navbar__item:not(.navbar__item--right)');
      navbarItems.forEach(async (item) => {
        if (item && !(item as HTMLElement).querySelector('button')) {
          storeOriginalContent(item as HTMLElement);
          await translateElement(item as HTMLElement);
        }
      });
    } catch (error) {
      console.error('Translation error:', error);
    } finally {
      setIsTranslating(false);
    }
  };

  const restoreEnglish = () => {
    // Restore main content
    const mainContent = document.querySelector('main.docMainContainer_node_modules-\\@docusaurus-theme-classic-lib-theme-DocPage-Layout-Main-styles-module') 
      || document.querySelector('main')
      || document.querySelector('.markdown');
    
    if (mainContent) {
      restoreOriginalContent(mainContent as HTMLElement);
    }

    // Restore sidebar
    const sidebar = document.querySelector('aside.theme-doc-sidebar-container');
    if (sidebar) {
      restoreOriginalContent(sidebar as HTMLElement);
    }

    // Restore navbar
    const navbarItems = document.querySelectorAll('.navbar__item:not(.navbar__item--right)');
    navbarItems.forEach((item) => {
      if (item) {
        restoreOriginalContent(item as HTMLElement);
      }
    });
  };

  const handleToggle = async () => {
    if (isLoading) return;

    // Require authentication before allowing translation
    if (!isAuthenticated) {
      setShowAuthModal(true);
      return;
    }

    // If currently translating and user clicks again, cancel
    if (isTranslationInProgress()) {
      cancelTranslation();
      return;
    }

    const newLang: Language = language === 'en' ? 'ur' : 'en';
    setLanguage(newLang);
    setPreferredLanguage(newLang);

    if (newLang === 'ur') {
      await applyTranslation();
    } else {
      // Turning off Urdu -> restore English
      restoreEnglish();
      // Force page reload to get original English content to ensure full restore
      window.location.reload();
    }
  };

  return (
    <>
      <div className="language-toggle-container">
        <button
          className={`language-toggle-button slim ${
            isTranslating ? 'translating' : language === 'ur' ? 'on state-urd' : 'off state-eng'
          } ${!isAuthenticated ? 'locked' : ''}`}
          onClick={handleToggle}
          title={
            !isAuthenticated
              ? 'Sign in to enable translation'
              : isTranslating
                ? 'Click to cancel translation'
                : language === 'en'
                  ? 'Switch to Urdu'
                  : 'Switch to English'
          }
          aria-disabled={!isAuthenticated}
        >
          {isTranslating ? (
            <span className="slim-cancel"><span className="spinner"></span>Cancel</span>
          ) : (
            <>
              <span className="toggle-track" aria-hidden="true" />
              <span className="toggle-knob" aria-hidden="true" />
              <span className="toggle-labels" aria-hidden="true">
                <span className="label-eng">ENG</span>
                <span className="label-urd">URD</span>
              </span>
            </>
          )}
        </button>
      </div>

      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onSuccess={() => setShowAuthModal(false)}
      />
    </>
  );
}
