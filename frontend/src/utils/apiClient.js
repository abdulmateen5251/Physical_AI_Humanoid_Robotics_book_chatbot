/**
 * API Client Configuration
 * Handles all communication with the backend REST API
 */

const API_BASE_URL = (typeof process !== 'undefined' && process.env?.REACT_APP_API_BASE_URL) || 'http://localhost:8090';
const API_TIMEOUT = parseInt((typeof process !== 'undefined' && process.env?.REACT_APP_API_TIMEOUT) || '30000', 10);

/**
 * Fetch wrapper with timeout and error handling
 */
const apiFetch = async (endpoint, options = {}) => {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), API_TIMEOUT);

  try {
    const url = `${API_BASE_URL}${endpoint}`;
    const response = await fetch(url, {
      ...options,
      headers: {
        'Content-Type': 'application/json',
        ...options.headers,
      },
      signal: controller.signal,
    });

    clearTimeout(timeout);

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `API Error: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    clearTimeout(timeout);
    if (error.name === 'AbortError') {
      throw new Error(`Request timeout after ${API_TIMEOUT}ms`);
    }
    throw error;
  }
};

/**
 * API Endpoints
 */
export const apiClient = {
  /**
   * Health check endpoint
   * GET /health
   */
  health: () => apiFetch('/health'),

  /**
   * Retrieve relevant documents from the knowledge base
   * POST /api/retrieve
   */
  retrieve: (query, topK = 5) =>
    apiFetch('/api/retrieve', {
      method: 'POST',
      body: JSON.stringify({ query, top_k: topK }),
    }),

  /**
   * Get AI-generated answer with citations
   * POST /api/answer
   */
  answer: (question, conversationHistory = []) =>
    apiFetch('/api/answer', {
      method: 'POST',
      body: JSON.stringify({
        question,
        conversation_history: conversationHistory,
      }),
    }),

  /**
   * Submit feedback on answer quality
   * POST /api/feedback
   */
  feedback: (questionId, answerId, rating, feedback = '') =>
    apiFetch('/api/feedback', {
      method: 'POST',
      body: JSON.stringify({
        question_id: questionId,
        answer_id: answerId,
        rating,
        feedback,
      }),
    }),
};

export default apiClient;
