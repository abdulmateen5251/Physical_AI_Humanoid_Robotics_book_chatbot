/**
 * Custom React hooks for API integration
 */

import { useState, useCallback, useRef } from 'react';
import apiClient from './apiClient';

/**
 * Hook for retrieving documents from knowledge base
 */
export const useRetrieve = () => {
  const [data, setData] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const retrieve = useCallback(async (query, topK = 5) => {
    setLoading(true);
    setError(null);
    try {
      const result = await apiClient.retrieve(query, topK);
      setData(result);
      return result;
    } catch (err) {
      const errorMsg = err.message || 'Failed to retrieve documents';
      setError(errorMsg);
      console.error('Retrieve error:', err);
      throw err;
    } finally {
      setLoading(false);
    }
  }, []);

  return { data, loading, error, retrieve };
};

/**
 * Hook for getting AI-generated answers
 */
export const useAnswer = () => {
  const [data, setData] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const conversationHistoryRef = useRef([]);

  const answer = useCallback(async (question) => {
    setLoading(true);
    setError(null);
    try {
      const result = await apiClient.answer(question, conversationHistoryRef.current);
      
      // Add to conversation history
      conversationHistoryRef.current.push({
        role: 'user',
        content: question,
      });
      conversationHistoryRef.current.push({
        role: 'assistant',
        content: result.answer,
      });

      setData(result);
      return result;
    } catch (err) {
      const errorMsg = err.message || 'Failed to get answer';
      setError(errorMsg);
      console.error('Answer error:', err);
      throw err;
    } finally {
      setLoading(false);
    }
  }, []);

  const clearHistory = useCallback(() => {
    conversationHistoryRef.current = [];
    setData(null);
    setError(null);
  }, []);

  return { data, loading, error, answer, clearHistory };
};

/**
 * Hook for submitting feedback
 */
export const useFeedback = () => {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [success, setSuccess] = useState(false);

  const submit = useCallback(async (questionId, answerId, rating, feedback = '') => {
    setLoading(true);
    setError(null);
    setSuccess(false);
    try {
      await apiClient.feedback(questionId, answerId, rating, feedback);
      setSuccess(true);
      return true;
    } catch (err) {
      const errorMsg = err.message || 'Failed to submit feedback';
      setError(errorMsg);
      console.error('Feedback error:', err);
      throw err;
    } finally {
      setLoading(false);
    }
  }, []);

  return { loading, error, success, submit };
};

/**
 * Hook for checking backend health
 */
export const useHealthCheck = () => {
  const [healthy, setHealthy] = useState(false);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  const check = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const result = await apiClient.health();
      setHealthy(result.status === 'ok');
      return result.status === 'ok';
    } catch (err) {
      setHealthy(false);
      const errorMsg = err.message || 'Health check failed';
      setError(errorMsg);
      console.error('Health check error:', err);
      return false;
    } finally {
      setLoading(false);
    }
  }, []);

  return { healthy, loading, error, check };
};
