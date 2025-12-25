// API Configuration for Docusaurus RAG Chatbot
const API_CONFIG = {
  // Base URL for the RAG API - this should be configured based on environment
  BASE_URL: (typeof process !== 'undefined' && process.env ? process.env.RAG_API_BASE_URL || process.env.REACT_APP_RAG_API_URL : null) || 'http://localhost:8000',

  // API endpoints
  ENDPOINTS: {
    QUESTION: '/question',
  },

  // Request timeout in milliseconds
  REQUEST_TIMEOUT: 30000, // 30 seconds

  // Maximum number of results to retrieve
  DEFAULT_TOP_K: 5,

  // Context timeout for selected text (in milliseconds)
  CONTEXT_TIMEOUT: 300000, // 5 minutes
};

export default API_CONFIG;