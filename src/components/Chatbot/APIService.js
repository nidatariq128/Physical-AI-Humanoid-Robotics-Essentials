// APIService.js - Service for communicating with the RAG backend API
import API_CONFIG from './config';

class APIService {
  constructor() {
    this.baseURL = API_CONFIG.BASE_URL;
    this.timeout = API_CONFIG.REQUEST_TIMEOUT;
  }

  // Set a new API endpoint if needed
  setAPIEndpoint(endpoint) {
    this.baseURL = endpoint;
  }

  // Set timeout for requests
  setTimeout(timeoutMs) {
    this.timeout = timeoutMs;
  }

  // Send a query to the RAG backend
  async sendQuery(query, context = null, topK = API_CONFIG.DEFAULT_TOP_K) {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      // Prepare the request body with query and optional context
      const requestBody = {
        question: query,
        text_scope: context || null,  // Use the context as text_scope for the backend
        top_k: topK
      };

      const response = await fetch(`${this.baseURL}${API_CONFIG.ENDPOINTS.QUESTION}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // Additional headers can be added here if needed
        },
        body: JSON.stringify(requestBody),
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        // Try to get more specific error information from the response
        let errorMessage = `API request failed with status ${response.status}: ${response.statusText}`;
        try {
          const errorData = await response.json();
          if (errorData.detail) {
            errorMessage = `${errorMessage}. Details: ${errorData.detail}`;
          }
        } catch (e) {
          // If we can't parse the error response, use the original message
        }

        throw new Error(errorMessage);
      }

      const data = await response.json();
      return {
        answer: data.answer,
        citations: data.citations || [],
        grounded: data.grounded,
        execution_time_ms: data.execution_time_ms,
        retrieval_results_count: data.retrieval_results_count
      };
    } catch (error) {
      clearTimeout(timeoutId);

      // Handle timeout specifically
      if (error.name === 'AbortError') {
        throw new Error('Request timeout: The API request took too long to complete');
      }

      // Re-throw the error for the calling function to handle
      throw error;
    }
  }
}

// Export a singleton instance
const apiService = new APIService();
export default apiService;

// Export the class as well for testing purposes if needed
export { APIService };