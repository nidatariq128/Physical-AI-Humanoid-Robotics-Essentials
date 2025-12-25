import React from 'react';
import BrowserOnlyChatbot from '../components/Chatbot/BrowserOnlyChatbot';
import { TranslationProvider } from '../context/TranslationContext';

// Default wrapper for the site
const Root = ({children}) => {
  return (
    <TranslationProvider>
      <>
        {children}
        <BrowserOnlyChatbot />
      </>
    </TranslationProvider>
  );
};

export default Root;