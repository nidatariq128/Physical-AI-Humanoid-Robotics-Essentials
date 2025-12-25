import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

const BrowserOnlyChatbot = () => {
  return (
    <BrowserOnly>
      {() => {
        const [ChatbotUIComponent, setChatbotUIComponent] = React.useState(null);

        React.useEffect(() => {
          const loadChatbotUI = async () => {
            const { default: ChatbotUI } = await import('./ChatbotUI');
            setChatbotUIComponent(() => ChatbotUI);
          };

          loadChatbotUI();
        }, []);

        if (ChatbotUIComponent) {
          return (
            <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 1000 }}>
              <ChatbotUIComponent
                apiEndpoint={(typeof process !== 'undefined' && process.env ? process.env.REACT_APP_RAG_API_URL : null) || 'http://localhost:8000'}
                initialOpen={false}
              />
            </div>
          );
        }

        return null;
      }}
    </BrowserOnly>
  );
};

export default BrowserOnlyChatbot;