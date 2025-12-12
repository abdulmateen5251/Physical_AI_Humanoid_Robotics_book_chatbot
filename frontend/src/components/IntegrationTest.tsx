/**
 * Integration Test: Backend-Frontend Connection
 * Verifies all API endpoints are properly connected
 */

import React, { useEffect, useState } from 'react';
import { useHealthCheck, useRetrieve, useAnswer } from '../utils/useApi';

interface TestResults {
  health: string | null;
  retrieve: string | null;
  answer: string | null;
}

export function IntegrationTest() {
  const [results, setResults] = useState<TestResults>({
    health: null,
    retrieve: null,
    answer: null,
  });

  const healthCheck = useHealthCheck();
  const { retrieve } = useRetrieve();
  const { answer } = useAnswer();

  useEffect(() => {
    const runTests = async () => {
      try {
        // Test 1: Health check
        const healthOk = await healthCheck.check();
        setResults(prev => ({
          ...prev,
          health: healthOk ? 'PASS ✅' : 'FAIL ❌',
        }));

        if (healthOk) {
          // Test 2: Retrieve documents
          try {
            const retrieveResult = await retrieve('ROS 2 topics');
            setResults(prev => ({
              ...prev,
              retrieve: `PASS ✅ (${(retrieveResult as any)?.results?.length || 0} documents)`,
            }));
          } catch (err) {
            const errorMsg = err instanceof Error ? err.message : 'Unknown error';
            setResults(prev => ({
              ...prev,
              retrieve: `FAIL ❌: ${errorMsg}`,
            }));
          }

          // Test 3: Get answer
          try {
            const answerResult = await answer('What is ROS 2?');
            setResults(prev => ({
              ...prev,
              answer: `PASS ✅ (${(answerResult as any)?.citations?.length || 0} citations)`,
            }));
          } catch (err) {
            const errorMsg = err instanceof Error ? err.message : 'Unknown error';
            setResults(prev => ({
              ...prev,
              answer: `FAIL ❌: ${errorMsg}`,
            }));
          }
        }
      } catch (err) {
        console.error('Integration test error:', err);
      }
    };

    runTests();
  }, [healthCheck, retrieve, answer]);

  return (
    <div style={{ padding: '20px', fontFamily: 'monospace', background: '#f5f5f5', borderRadius: '8px', margin: '20px 0' }}>
      <h3>🔗 Backend-Frontend Integration Status</h3>
      <div style={{ display: 'grid', gap: '10px' }}>
        <div>
          <strong>Health Check:</strong> {results.health || '⏳ Testing...'}
        </div>
        <div>
          <strong>Retrieve Documents:</strong> {results.retrieve || '⏳ Testing...'}
        </div>
        <div>
          <strong>Get Answer:</strong> {results.answer || '⏳ Testing...'}
        </div>
      </div>
    </div>
  );
}

export default IntegrationTest;
