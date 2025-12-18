import React, { useState, useEffect } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import { useAuth } from './AuthContext';
import AuthModal from './AuthModal';
import ProtectedContent from './ProtectedContent';
import './ChatWidget.css';

interface SelectionData {
  text: string;
  chapter?: string;
  section?: string;
  page?: string;
}

interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  citations?: Array<{
    chapter?: string;
    section?: string;
    page?: string;
    uri?: string;
  }>;
}

const ChatWidget: React.FC = () => {
  const { isAuthenticated } = useAuth();
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [selection, setSelection] = useState<SelectionData | null>(null);
  const [selectionOnly, setSelectionOnly] = useState(false);
  const [sessionId] = useState(() => {
    // Generate unique session ID
    return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  });
  const [isVisible, setIsVisible] = useState(false);
  const [showAuthModal, setShowAuthModal] = useState(false);

  // Capture text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText && selectedText.length > 10) {
        // Extract metadata from current page
        const chapterElement = document.querySelector('[data-chapter]');
        const sectionElement = document.querySelector('[data-section]');
        
        setSelection({
          text: selectedText,
          chapter: chapterElement?.getAttribute('data-chapter') || undefined,
          section: sectionElement?.getAttribute('data-section') || undefined,
        });
      } else {
        // Clear selection if nothing is selected
        setSelection(null);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const sendMessage = async () => {
    if (!input.trim() || !isAuthenticated) return;

    // Save input before clearing
    const question = input;
    
    const userMessage: ChatMessage = {
      role: 'user',
      content: question
    };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);
    
    // Save current selection state
    const currentSelection = selection;
    const currentSelectionOnly = selectionOnly;

    try {
      // Only send selection if it has valid text content
      const validSelection = currentSelection?.text ? currentSelection : null;
      const shouldUseSelectionOnly = currentSelectionOnly && !!validSelection;

      console.log('Sending message to backend:', {
        question: question,
        session_id: sessionId,
        selection_only: shouldUseSelectionOnly,
        selection: validSelection
      });

      const response = await fetch('http://localhost:8090/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          question: question,
          session_id: sessionId,
          selection_only: shouldUseSelectionOnly,
          selection: validSelection
        })
      });

      console.log('Response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('Backend error:', errorText);
        throw new Error(`Backend returned ${response.status}: ${errorText}`);
      }

      const data = await response.json();
      console.log('Received data:', data);

      const assistantMessage: ChatMessage = {
        role: 'assistant',
        content: data.answer || 'No answer received',
        citations: data.citations || []
      };
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage: ChatMessage = {
        role: 'assistant',
        content: `Error: ${error instanceof Error ? error.message : 'Could not connect to chatbot service. Make sure backend is running on http://localhost:8090'}`
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div>
      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onSuccess={() => setShowAuthModal(false)}
      />

      {!isVisible && (
        <button className="chat-floating-button" onClick={() => setIsVisible(true)}>
          💬
        </button>
      )}
      
      {isVisible && (
        <div className="chat-container">
          <div className="chat-header">
            <div className="chat-header-top">
              <h3 className="chat-title">Book Q&A Chatbot</h3>
              <div className="chat-header-buttons">
                <button className="chat-icon-button" onClick={() => setIsVisible(false)} title="Minimize">
                  −
                </button>
              </div>
            </div>
            {selection && isAuthenticated && (
              <div className="chat-selection-info">
                <label className="chat-checkbox-label">
                  <input
                    type="checkbox"
                    checked={selectionOnly}
                    onChange={(e) => setSelectionOnly(e.target.checked)}
                  />
                  <span>Answer from selection only</span>
                </label>
                <div className="chat-selection-text">
                  Selected: "{selection.text.substring(0, 50)}..."
                </div>
              </div>
            )}
          </div>

          <ProtectedContent onLoginRequired={() => setShowAuthModal(true)}>
            <div className="chat-messages">
              {messages.map((msg, idx) => (
                <div key={idx} className={msg.role === 'user' ? 'chat-message-user' : 'chat-message-assistant'}>
                  <strong>{msg.role === 'user' ? 'You' : 'Assistant'}:</strong>
                  <div className="chat-markdown chat-message-content">
                    <ReactMarkdown remarkPlugins={[remarkGfm]}>
                      {msg.content}
                    </ReactMarkdown>
                  </div>
                  {msg.citations && msg.citations.length > 0 && (
                    <div className="chat-citations">
                      <strong>Sources:</strong>
                      <ul>
                        {msg.citations.map((c, i) => (
                          <li key={i}>
                            {c.chapter && `Chapter: ${c.chapter}`}
                            {c.section && `, Section: ${c.section}`}
                            {c.page && `, Page: ${c.page}`}
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}
                </div>
              ))}
              {loading && (
                <div className="chat-loading">
                  <div className="chat-loading-dots">
                    <span className="dot"></span>
                    <span className="dot"></span>
                    <span className="dot"></span>
                    <span className="dot"></span>
                  </div>
                </div>
              )}
            </div>

            <div className="chat-input-container">
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
                placeholder="Ask a question about the book..."
                className="chat-input"
                disabled={!isAuthenticated}
              />
              <button onClick={sendMessage} disabled={loading || !isAuthenticated} className="chat-button">
                Send
              </button>
            </div>
          </ProtectedContent>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
