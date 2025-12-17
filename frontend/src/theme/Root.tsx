/**
 * Root Component - Wraps entire app with AuthProvider
 */

import React from 'react';
import { AuthProvider } from '../components/AuthContext';
import ChatWidget from '../components/ChatWidget';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <AuthProvider>
      {children}
      <ChatWidget />
    </AuthProvider>
  );
}
