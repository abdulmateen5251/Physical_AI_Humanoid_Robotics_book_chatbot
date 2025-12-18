import React, { useEffect, useState } from 'react';
import { getPreferredLanguage } from '../utils/translation';
import LanguageToggle from './LanguageToggle';
import './DocPageTranslation.css';

export default function DocPageTranslation() {
  const [currentLang, setCurrentLang] = useState<'en' | 'ur'>('en');

  useEffect(() => {
    // Update language state when it changes
    const handleLanguageChange = (e: CustomEvent) => {
      setCurrentLang(e.detail.language);
    };

    window.addEventListener('languageChange', handleLanguageChange as EventListener);
    
    // Set initial language
    setCurrentLang(getPreferredLanguage());

    return () => {
      window.removeEventListener('languageChange', handleLanguageChange as EventListener);
    };
  }, []);

  return (
    <div className="doc-page-translation">
      <div className="doc-page-translation-banner">
        <div className="translation-info">
          <span className="translation-icon">🌐</span>
          <span className="translation-text">
            {currentLang === 'en' 
              ? 'Read this chapter in Urdu by clicking the language toggle button in the navbar ↗️' 
              : 'اس باب کو انگریزی میں پڑھنے کے لیے نیویگیشن بار میں زبان کا بٹن دبائیں'}
          </span>
        </div>
        <div className="translation-button-wrapper">
          <LanguageToggle />
        </div>
      </div>
    </div>
  );
}
