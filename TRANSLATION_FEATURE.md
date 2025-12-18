# Translation Feature Documentation

## Overview
A complete English to Urdu translation system that can translate every chapter and saves the user's language preference.

## Features

### 1. **Navbar Toggle Button** 🌐
- There is a language toggle button in the navbar
- Clicking switches between English ↔ Urdu
- The current language is displayed on the button (🇬🇧 English or 🇵🇰 اردو)
- Translation progress indicator is also included

### 2. **Chapter Banner** 📖
- There is a translation banner at the start of each chapter
- Informs the user that Urdu translation is available
- Quick access button for translation

### 3. **Smart Translation** 🧠
- Code blocks are not translated (safe!)
- Translation is cached (fast!)
- Batch processing avoids rate limiting
- Progress indicator shows the user what's happening

### 4. **Persistent Preference** 💾
- User's language choice is saved in localStorage
- Urdu remains even after page refresh
- Preference is saved even after closing the browser

### 5. **Toggle & Cancel** 🔁
- Clear on/off states shown as "Urdu On/Urdu Off"
- Click the toggle while "Translating..." to cancel immediately
- Visual switch indicator for the toggle state
- Emits start/end/cancel/progress events to keep UI responsive

## Technical Implementation

### Files Created/Modified

#### 1. Translation Utility
**Path:** `frontend/src/utils/translation.ts`
- Core translation logic
- MyMemory Translation API integration
- Caching system
- Content restoration
 - Cancelable translation via `AbortController`
 - Concurrent batching for faster translation

#### 2. Language Toggle Component
**Path:** `frontend/src/components/LanguageToggle.tsx`
- Interactive toggle button
- Translation trigger
- Loading state management
- Auto-translation on page load

#### 3. Doc Page Banner
**Path:** `frontend/src/components/DocPageTranslation.tsx`
- Chapter-level translation banner
- User guidance
- Quick translation access

#### 4. Navbar Integration
**Path:** `frontend/src/theme/Navbar/Content/index.tsx`
- Language toggle button in navbar
- Responsive positioning

#### 5. Doc Layout Wrapper
**Path:** `frontend/src/theme/DocItem/Layout/index.tsx`
- Adds translation banner to every doc page

### CSS Files
- `LanguageToggle.css` - Toggle button styling
- `DocPageTranslation.css` - Banner styling
- Mobile responsive styles included

## How It Works

### Translation Flow
```
1. User clicks language toggle
   ↓
2. Language preference saved to localStorage
   ↓
3. DOM traversal finds all text nodes
   ↓
4. Original content backed up
   ↓
5. Text sent to MyMemory API in batches
   ↓
6. Translations cached in localStorage
   ↓
7. DOM updated with Urdu text
   ↓
8. Progress events dispatched
```

### Caching Strategy
```typescript
localStorage:
  - preferred_language: 'en' | 'ur'
  - translation_cache: { [englishText]: urduTranslation }
  - original_content: { [elementPath]: originalHTML }
```

## API Details

### Translation API
**Provider:** MyMemory Translated
**Endpoint:** `https://api.mymemory.translated.net/get`
**Free Tier:** 1000 words/day per IP
**Alternative:** Can integrate Google Cloud Translation API

### API Request Format
```javascript
const apiUrl = `https://api.mymemory.translated.net/get?q=${text}&langpair=en|ur`;
```

## Usage Instructions

### For Users
1. **Navbar Toggle:**
   - Click the 🇬🇧 English button in the navbar to turn Urdu on
   - Translation starts; the button shows "Translating..."
   - Click again while it is translating to cancel immediately
   - When Urdu is on, the button shows "Urdu On" with 🇵🇰

2. **Chapter Banner:**
   - There is a banner at the top of each chapter
   - Click the "Read in Urdu" button
   - Get instant translation

3. **Revert to English:**
   - Click the toggle to turn Urdu off
   - Original English content is restored and the page reloads to ensure full revert

### For Developers

#### Adding Translation to New Components
```tsx
import { translateElement, storeOriginalContent } from '@site/src/utils/translation';

// Store original before translating
storeOriginalContent(elementRef.current);

// Translate
await translateElement(elementRef.current);
```

#### Listening to Language Changes
```tsx
useEffect(() => {
  const handleLangChange = (e: CustomEvent) => {
    console.log('New language:', e.detail.language);
  };
  
  window.addEventListener('languageChange', handleLangChange);
  return () => window.removeEventListener('languageChange', handleLangChange);
}, []);
```

#### Handling Translation Events (start/end/cancel/progress)
```tsx
useEffect(() => {
   const onStart = () => console.log('Translation started');
   const onEnd = () => console.log('Translation finished');
   const onCanceled = () => console.log('Translation canceled');
   const onProgress = (e: CustomEvent) => console.log('Progress:', e.detail.progress + '%');

   window.addEventListener('translationStart', onStart as EventListener);
   window.addEventListener('translationEnd', onEnd as EventListener);
   window.addEventListener('translationCanceled', onCanceled as EventListener);
   window.addEventListener('translationProgress', onProgress as EventListener);

   return () => {
      window.removeEventListener('translationStart', onStart as EventListener);
      window.removeEventListener('translationEnd', onEnd as EventListener);
      window.removeEventListener('translationCanceled', onCanceled as EventListener);
      window.removeEventListener('translationProgress', onProgress as EventListener);
   };
}, []);
```

#### Progress Tracking
```tsx
useEffect(() => {
  const handleProgress = (e: CustomEvent) => {
    console.log('Translation progress:', e.detail.progress + '%');
  };
  
  window.addEventListener('translationProgress', handleProgress);
  return () => window.removeEventListener('translationProgress', handleProgress);
}, []);
```

## Mobile Responsive

### Phone Layout (< 480px)
- Toggle button shows only flag icon
- Full-width banner in chapters
- Optimized touch targets

### Tablet Layout (< 768px)
- Compact button with text
- Stacked banner layout
- Better spacing

## Dark Mode Support
- Full dark mode compatibility
- Adjusted colors for readability
- Gradient buttons maintain visibility

## Performance Optimizations

1. **Caching:** Already translated text reused
2. **Concurrent Batching:** Multiple texts translated in parallel (limited workers)
3. **Rate Limiting Friendly:** Batches sent with controlled concurrency
4. **Smart Detection:** Code blocks ignored
5. **Progress Feedback:** User knows what's happening
6. **Cancelable:** Aborts in-flight requests when user cancels

## Future Enhancements

### Potential Improvements
- [ ] Add more languages (Arabic, Hindi, etc.)
- [ ] Offline translation with local dictionary
- [ ] Google Cloud Translation API integration
- [ ] Translation quality feedback
- [ ] Paragraph-level caching (better than word-level)
- [ ] Custom translation overrides
- [ ] Admin panel for translation management

### API Upgrade Options
1. **Google Cloud Translation:**
   - Better quality
   - Higher limits
   - Paid service
   
2. **LibreTranslate:**
   - Open source
   - Self-hosted option
   - No limits

3. **DeepL API:**
   - Best quality
   - Limited free tier
   - Good for technical content

## Troubleshooting

### Translation Not Working
1. Check internet connection
2. Clear localStorage: `localStorage.clear()`
3. Check browser console for errors
4. API rate limit exceeded (wait 24 hours)

### Content Not Saving
1. Check localStorage quota
2. Private browsing mode may block localStorage
3. Browser settings may disable storage

### Partial Translation
1. API rate limit reached
2. Some text in code blocks (intentionally skipped)
3. Refresh and try again

## Testing

### Manual Testing Checklist
- [ ] Toggle button shows in navbar
- [ ] Banner appears in each chapter
- [ ] Translation works on button click
- [ ] Preference persists after refresh
- [ ] Code blocks remain in English
- [ ] Mobile responsive works
- [ ] Dark mode looks good
- [ ] Progress indicator shows
- [ ] Revert to English works

### Browser Compatibility
- ✅ Chrome/Edge (latest)
- ✅ Firefox (latest)
- ✅ Safari (latest)
- ✅ Mobile browsers

## Support
If you have any issues or need improvements, please create a GitHub issue.

---
**Last Updated:** December 18, 2025
**Version:** 1.0.0
