/**
 * Translation utility for English to Urdu translation
 * Uses localStorage to persist language preference
 */

export type Language = 'en' | 'ur';

const STORAGE_KEY = 'preferred_language';
const TRANSLATION_CACHE_KEY = 'translation_cache';
const ORIGINAL_CONTENT_KEY = 'original_content';

// Translation state
let currentController: AbortController | null = null;
let translating = false;

export function isTranslationInProgress(): boolean {
  return translating;
}

export function cancelTranslation(): void {
  if (currentController) {
    currentController.abort();
  }
}

// Get saved language preference
export function getPreferredLanguage(): Language {
  if (typeof window === 'undefined') return 'en';
  return (localStorage.getItem(STORAGE_KEY) as Language) || 'en';
}

// Save language preference
export function setPreferredLanguage(lang: Language): void {
  if (typeof window === 'undefined') return;
  localStorage.setItem(STORAGE_KEY, lang);
  
  // Dispatch custom event to notify components
  window.dispatchEvent(new CustomEvent('languageChange', { detail: { language: lang } }));
}

// Get translation cache
function getTranslationCache(): Record<string, string> {
  if (typeof window === 'undefined') return {};
  const cached = localStorage.getItem(TRANSLATION_CACHE_KEY);
  return cached ? JSON.parse(cached) : {};
}

// Save translation to cache
function saveToCache(text: string, translation: string): void {
  if (typeof window === 'undefined') return;
  const cache = getTranslationCache();
  cache[text] = translation;
  localStorage.setItem(TRANSLATION_CACHE_KEY, JSON.stringify(cache));
}

// Get from cache
function getFromCache(text: string): string | null {
  const cache = getTranslationCache();
  return cache[text] || null;
}

/**
 * Translate text from English to Urdu using Google Translate API
 * Falls back to cached translation if API fails
 */
export async function translateToUrdu(text: string, signal?: AbortSignal): Promise<string> {
  if (!text || text.trim() === '') return text;
  
  // Don't translate if text is too short or looks like code
  if (text.length < 2 || /^[\d\s\{\}\[\]\(\)]+$/.test(text)) return text;
  
  // Check cache first
  const cached = getFromCache(text);
  if (cached) return cached;

  try {
    // Using LibreTranslate API or MyMemory API as fallback
    // You can also integrate Google Cloud Translation API here
    const apiUrl = `https://api.mymemory.translated.net/get?q=${encodeURIComponent(text)}&langpair=en|ur`;
    
    const response = await fetch(apiUrl, { signal });
    const data = await response.json();
    
    if (data.responseData && data.responseData.translatedText) {
      const translation = data.responseData.translatedText;
      saveToCache(text, translation);
      return translation;
    }
    
    return text; // Return original if translation fails
  } catch (error) {
    // If aborted, just return original text silently
    if ((error as any)?.name === 'AbortError') {
      return text;
    }
    console.error('Translation error:', error);
    return text;
  }
}

/**
 * Translate multiple texts with limited concurrency for speed
 */
export async function translateBatch(
  texts: string[],
  options: { concurrency?: number; signal?: AbortSignal } = {}
): Promise<string[]> {
  const { concurrency = 4, signal } = options;
  const results: string[] = new Array(texts.length);

  let index = 0;

  async function worker(workerId: number) {
    while (index < texts.length) {
      const i = index++;
      if (signal?.aborted) return;
      results[i] = await translateToUrdu(texts[i], signal);
    }
  }

  const workers = Array.from({ length: Math.min(concurrency, texts.length) }, (_, w) => worker(w));
  await Promise.all(workers);
  return results;
}

/**
 * Translate all text nodes in a DOM element
 */
export async function translateElement(element: HTMLElement): Promise<void> {
  const walker = document.createTreeWalker(
    element,
    NodeFilter.SHOW_TEXT,
    null
  );

  const textNodes: { node: Text; original: string }[] = [];
  let node: Node | null;
  
  while ((node = walker.nextNode())) {
    const textNode = node as Text;
    const text = textNode.textContent?.trim();
    if (text && text.length > 0 && !isCode(textNode)) {
      textNodes.push({ 
        node: textNode, 
        original: textNode.textContent || '' 
      });
    }
  }

  // Store original content
  saveOriginalContent(element);

  // Prepare abort controller and state
  currentController = new AbortController();
  translating = true;
  const { signal } = currentController;
  window.dispatchEvent(new CustomEvent('translationStart'));

  try {
    // Translate all text nodes with batching + concurrency
    const batchSize = 20; // larger batches
    const concurrency = 4;
    for (let i = 0; i < textNodes.length; i += batchSize) {
      if (signal.aborted) break;
      const batch = textNodes.slice(i, i + batchSize);
      const texts = batch.map(item => item.original);
      const translations = await translateBatch(texts, { concurrency, signal });

      if (signal.aborted) break;
      batch.forEach((item, index) => {
        item.node.textContent = translations[index];
      });

      // Show progress
      const progress = Math.min(100, Math.round(((i + batchSize) / textNodes.length) * 100));
      window.dispatchEvent(new CustomEvent('translationProgress', { detail: { progress } }));
    }
  } finally {
    const wasAborted = signal.aborted;
    translating = false;
    window.dispatchEvent(new CustomEvent(wasAborted ? 'translationCanceled' : 'translationEnd'));
    currentController = null;
  }
}

/**
 * Check if a text node is inside a code block
 */
function isCode(node: Node): boolean {
  let parent = node.parentElement;
  while (parent) {
    const tagName = parent.tagName.toLowerCase();
    if (['code', 'pre', 'script', 'style', 'svg'].includes(tagName)) {
      return true;
    }
    if (parent.classList.contains('prism-code') || parent.classList.contains('token')) {
      return true;
    }
    parent = parent.parentElement;
  }
  return false;
}

/**
 * Store original content before translation
 */
function saveOriginalContent(element: HTMLElement): void {
  if (typeof window === 'undefined') return;
  
  const path = getElementPath(element);
  const originalContent = localStorage.getItem(ORIGINAL_CONTENT_KEY);
  const contentMap = originalContent ? JSON.parse(originalContent) : {};
  
  if (!contentMap[path]) {
    contentMap[path] = element.innerHTML;
    localStorage.setItem(ORIGINAL_CONTENT_KEY, JSON.stringify(contentMap));
  }
}

/**
 * Restore original content
 */
export function restoreOriginalContent(element: HTMLElement): void {
  if (typeof window === 'undefined') return;
  
  const path = getElementPath(element);
  const originalContent = localStorage.getItem(ORIGINAL_CONTENT_KEY);
  
  if (originalContent) {
    const contentMap = JSON.parse(originalContent);
    if (contentMap[path]) {
      element.innerHTML = contentMap[path];
    }
  }
}

/**
 * Get a unique path for an element
 */
function getElementPath(element: HTMLElement): string {
  const parts: string[] = [];
  let current: HTMLElement | null = element;
  
  while (current && current !== document.body) {
    let selector = current.tagName.toLowerCase();
    if (current.id) {
      selector += `#${current.id}`;
    } else if (current.className) {
      selector += `.${current.className.split(' ')[0]}`;
    }
    parts.unshift(selector);
    current = current.parentElement;
  }
  
  return parts.join(' > ');
}

/**
 * Store original content before translation
 */
const originalContentMap = new Map<HTMLElement, string>();

export function storeOriginalContent(element: HTMLElement): void {
  if (!originalContentMap.has(element)) {
    originalContentMap.set(element, element.innerHTML);
  }
  saveOriginalContent(element);
}

/**
 * Clear all translation caches
 */
export function clearTranslationCache(): void {
  if (typeof window === 'undefined') return;
  localStorage.removeItem(TRANSLATION_CACHE_KEY);
  localStorage.removeItem(ORIGINAL_CONTENT_KEY);
  originalContentMap.clear();
}
