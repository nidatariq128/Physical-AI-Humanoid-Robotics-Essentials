import React from 'react';
import BrowserOnlyChatbot from '../../components/Chatbot/BrowserOnlyChatbot';

const LayoutWrapper = ({children, ...props}) => {
  return (
    <>
      <div style={{position: 'relative', minHeight: '100vh'}}>
        {children}
      </div>
      <BrowserOnlyChatbot />
    </>
  );
};

export default LayoutWrapper;