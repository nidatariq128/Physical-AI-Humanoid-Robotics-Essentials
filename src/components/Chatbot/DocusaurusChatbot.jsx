import React from 'react';
import ChatbotUI from './ChatbotUI';

const DocusaurusChatbot = () => {
  return (
    <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 1000 }}>
      <ChatbotUI
        apiEndpoint={(typeof process !== 'undefined' && process.env ? process.env.REACT_APP_RAG_API_URL : null) || 'http://localhost:8000'}
        initialOpen={false}
      />
    </div>
  );
};

export default DocusaurusChatbot;