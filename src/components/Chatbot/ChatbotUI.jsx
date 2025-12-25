import React, { useState, useRef, useEffect, useCallback, useMemo } from 'react';
import { useTranslation } from '../../context/TranslationContext';
import apiService from './APIService';
import textSelectionHandler from './TextSelectionHandler';
import { createLoadingState, createSuccessState, createErrorState, createIdleState, isLoading, hasError, hasData } from './stateUtils';
import './chatbot.css';
import ErrorBoundaryWrapper from './ErrorBoundaryWrapper';

const ChatbotUI = ({
  apiEndpoint = null,
  initialOpen = false,
  contextTimeout = 300000, // 5 minutes
  onResponse = null,
  onError = null
}) => {
  const { t } = useTranslation();
  const [isOpen, setIsOpen] = useState(initialOpen);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [requestState, setRequestState] = useState(createIdleState());
  const [selectedTextContext, setSelectedTextContext] = useState(null);
  const [contextExpiry, setContextExpiry] = useState(null);

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Initialize API endpoint if provided
  useEffect(() => {
    if (apiEndpoint) {
      apiService.setAPIEndpoint(apiEndpoint);
    }
  }, [apiEndpoint]);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Initialize text selection handler
  useEffect(() => {
    // Set up callback for when text is selected
    textSelectionHandler.onTextSelected((selectionData) => {
      setSelectedTextContext(selectionData.content);
      setContextExpiry(Date.now() + contextTimeout); // Set expiry time

      // Optionally show a visual indicator to the user
      console.log('Text selected for context:', selectionData.content.substring(0, 50) + '...');
    });

    // Set up callback for when selection is cleared
    textSelectionHandler.onSelectionCleared(() => {
      setSelectedTextContext(null);
      setContextExpiry(null);
    });

    // Start listening for text selection
    textSelectionHandler.startListening();

    // Cleanup function
    return () => {
      textSelectionHandler.stopListening();
    };
  }, [contextTimeout]);

  // Handle selected text context expiration
  useEffect(() => {
    if (contextExpiry) {
      const timer = setTimeout(() => {
        setSelectedTextContext(null);
        setContextExpiry(null);
      }, contextTimeout);

      return () => clearTimeout(timer);
    }
  }, [contextExpiry, contextTimeout]);

  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  const handleSendMessage = useCallback(async () => {
    if (!inputValue.trim()) return;

    const userMessage = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: new Date()
    };

    // Add user message to the chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');

    // Set loading state
    setRequestState(createLoadingState(t('chatbot.thinking') + '...'));

    try {
      // Use selected text context if it's still valid (not expired)
      const isContextValid = contextExpiry && Date.now() < contextExpiry;
      const contextToUse = isContextValid ? selectedTextContext : null;

      // Send query to API
      const response = await apiService.sendQuery(inputValue, contextToUse);

      // Handle empty response case
      let responseContent = response.answer;
      if (!responseContent || responseContent.trim() === '') {
        responseContent = t('chatbot.noRelevantInfo');
      }

      // Create bot message with citations
      const botMessage = {
        id: `bot-${Date.now()}`,
        role: 'bot',
        content: responseContent,
        timestamp: new Date(),
        citations: response.citations
      };

      // Add bot message to the chat
      setMessages(prev => [...prev, botMessage]);

      // Set success state
      setRequestState(createSuccessState(response));

      // Call onResponse callback if provided
      if (onResponse) {
        onResponse(response);
      }
    } catch (error) {
      console.error('Error sending message:', error);

      // Create error message
      const errorMessage = {
        id: `error-${Date.now()}`,
        role: 'bot',
        content: `${t('chatbot.error')}${error.message || t('chatbot.noRelevantInfo')}`,
        timestamp: new Date()
      };

      // Add error message to the chat
      setMessages(prev => [...prev, errorMessage]);

      // Set error state
      setRequestState(createErrorState(error.message || 'Unknown error'));

      // Call onError callback if provided
      if (onError) {
        onError(error);
      }
    }
  }, [inputValue, contextExpiry, selectedTextContext, contextTimeout, onResponse, onError]); // Add dependencies

  const handleKeyPress = useCallback((e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  }, [handleSendMessage]);

  const toggleChat = useCallback(() => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  }, [isOpen]);

  const closeChat = useCallback(() => {
    setIsOpen(false);
  }, []);

  const clearChat = useCallback(() => {
    setMessages([]);
  }, []);

  // Memoize the messages list to prevent unnecessary re-renders
  const renderedMessages = useMemo(() => {
    return messages.map((message) => (
      <div
        key={message.id}
        className={`chatbot-message ${message.role === 'user' ? 'message-user' : 'message-bot'}`}
      >
        {message.content}
        {message.citations && message.citations.length > 0 && (
          <div className="chatbot-citations">
            <div className="chatbot-citation">Sources:</div>
            {message.citations.map((citation, index) => (
              <div key={index} className="chatbot-citation">
                <a
                  href={citation.source}
                  target="_blank"
                  rel="noopener noreferrer"
                  onClick={(e) => {
                    // If the citation source looks like a local documentation link,
                    // handle it specially to navigate within the site
                    if (citation.source &&
                        (citation.source.startsWith('/') ||
                         citation.source.includes(window.location.hostname))) {
                      e.preventDefault();
                      window.open(citation.source, '_self');
                    }
                  }}
                >
                  {citation.source}
                </a>
              </div>
            ))}
          </div>
        )}
      </div>
    ));
  }, [messages]);

  return (
    <ErrorBoundaryWrapper>
      <div className="chatbot-container" role="region" aria-label="AI Assistant Chat Interface">
        {!isOpen ? (
          <button
            className="chatbot-toggle"
            onClick={toggleChat}
            aria-label={t('chatbot.open') || 'Open chatbot'}
            aria-expanded={false}
          >
            <span aria-hidden="true">💬</span>
          </button>
        ) : (
          <div className="chatbot-window" role="dialog" aria-modal="true" aria-labelledby="chatbot-title">
            <div className="chatbot-header">
              <h3 id="chatbot-title" className="chatbot-title">{t('chatbot.title') || 'AI Assistant'}</h3>
              <button
                className="chatbot-close"
                onClick={closeChat}
                aria-label={t('chatbot.close') || 'Close chat'}
                aria-describedby="chatbot-title"
              >
                ×
              </button>
            </div>

            <div
              className="chatbot-messages"
              role="log"
              aria-live="polite"
              aria-relevant="additions"
            >
              {messages.length === 0 && (
                <div className="chatbot-message message-bot" role="status" aria-live="polite">
                  {t('chatbot.welcome')}
                </div>
              )}

              {renderedMessages}

              {isLoading(requestState) && (
                <div className="chatbot-message message-bot" role="status" aria-live="assertive">
                  <div className="chatbot-loading" role="alert" aria-label="Processing your request">
                    <span>Thinking</span>
                    <div className="loading-dot" aria-hidden="true"></div>
                    <div className="loading-dot" aria-hidden="true"></div>
                    <div className="loading-dot" aria-hidden="true"></div>
                  </div>
                </div>
              )}

              {hasError(requestState) && (
                <div className="chatbot-error" role="alert" aria-live="assertive">
                  Error: {requestState.error}
                </div>
              )}

              <div ref={messagesEndRef} aria-hidden="true" />
            </div>

            <div className="chatbot-input-area" role="form" aria-label="Chat input area">
              <div style={{ flex: 1, display: 'flex', flexDirection: 'column' }}>
                {selectedTextContext && contextExpiry && Date.now() < contextExpiry && (
                  <div
                    className="chatbot-citation"
                    style={{ fontSize: '11px', marginBottom: '4px', color: '#1b6dfc' }}
                    role="status"
                    aria-live="polite"
                  >
                    {t('chatbot.context')}{selectedTextContext.length > 60 ? selectedTextContext.substring(0, 60) + '...' : selectedTextContext}"
                  </div>
                )}
                <textarea
                  ref={inputRef}
                  className="chatbot-input"
                  value={inputValue}
                  onChange={(e) => setInputValue(e.target.value)}
                  onKeyPress={handleKeyPress}
                  placeholder={selectedTextContext && contextExpiry && Date.now() < contextExpiry
                    ? t('chatbot.placeholder.context')
                    : t('chatbot.placeholder.default')}
                  rows={1}
                  style={{
                    height: 'auto',
                    minHeight: '40px',
                    maxHeight: '100px',
                    overflowY: 'auto'
                  }}
                  aria-label="Type your question here"
                  aria-describedby={selectedTextContext && contextExpiry && Date.now() < contextExpiry ? undefined : "chatbot-context-help"}
                />
                {selectedTextContext && contextExpiry && Date.now() < contextExpiry && (
                  <span id="chatbot-context-help" style={{display: 'none'}}>
                    Using selected text as context for your question
                  </span>
                )}
              </div>
              <button
                className="chatbot-send-button"
                onClick={handleSendMessage}
                disabled={!inputValue.trim() || isLoading(requestState)}
                aria-label="Send message"
                aria-disabled={!inputValue.trim() || isLoading(requestState)}
              >
                <span aria-hidden="true">➤</span>
              </button>
            </div>
          </div>
        )}
      </div>
    </ErrorBoundaryWrapper>
  );
};

export default React.memo(ChatbotUI);