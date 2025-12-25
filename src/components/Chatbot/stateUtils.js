// State management utilities for the chatbot

/**
 * Creates a loading state object
 * @param {string} message - Optional loading message
 * @returns {Object} Loading state object
 */
export const createLoadingState = (message = 'Processing...') => ({
  isLoading: true,
  loadingMessage: message,
  error: null,
  data: null
});

/**
 * Creates a success state object
 * @param {any} data - The data to store
 * @returns {Object} Success state object
 */
export const createSuccessState = (data) => ({
  isLoading: false,
  loadingMessage: null,
  error: null,
  data: data
});

/**
 * Creates an error state object
 * @param {Error|string} error - The error object or message
 * @returns {Object} Error state object
 */
export const createErrorState = (error) => ({
  isLoading: false,
  loadingMessage: null,
  error: error instanceof Error ? error.message : error,
  data: null
});

/**
 * Creates an idle state object
 * @returns {Object} Idle state object
 */
export const createIdleState = () => ({
  isLoading: false,
  loadingMessage: null,
  error: null,
  data: null
});

/**
 * Checks if a state represents a loading condition
 * @param {Object} state - The state object to check
 * @returns {boolean} True if loading
 */
export const isLoading = (state) => {
  return state && state.isLoading === true;
};

/**
 * Checks if a state represents an error condition
 * @param {Object} state - The state object to check
 * @returns {boolean} True if in error state
 */
export const hasError = (state) => {
  return state && state.error !== null;
};

/**
 * Checks if a state represents a success condition
 * @param {Object} state - The state object to check
 * @returns {boolean} True if in success state
 */
export const hasData = (state) => {
  return state && state.data !== null && !state.isLoading && !state.error;
};