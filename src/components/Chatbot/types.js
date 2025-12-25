/**
 * @typedef {Object} SourceCitation
 * @property {string} source - The source identifier (URL, document name, etc.)
 * @property {string} content - The content of the citation
 * @property {number} score - Relevance score
 * @property {Object} metadata - Additional metadata about the source
 */

/**
 * @typedef {Object} APIResponse
 * @property {string} answer - The AI-generated answer
 * @property {SourceCitation[]} citations - List of source citations
 * @property {boolean} grounded - Whether the response is grounded in the context
 * @property {number} execution_time_ms - Time taken to generate the response in milliseconds
 * @property {number} retrieval_results_count - Number of retrieved results used
 */

/**
 * @typedef {Object} ChatMessage
 * @property {string} id - Unique identifier for the message
 * @property {'user' | 'bot'} role - The role of the message sender
 * @property {string} content - The message content
 * @property {Date} timestamp - When the message was created
 * @property {SourceCitation[]} [citations] - Optional citations for bot responses
 */

/**
 * @typedef {Object} TextSelection
 * @property {string} content - The selected text content
 * @property {string} context - Additional context about where the text was selected
 * @property {Date} timestamp - When the selection was made
 */

export {}; // Required to make this file a module