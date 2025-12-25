// TextSelectionHandler.js - Utility for detecting and handling text selection

class TextSelectionHandler {
  constructor() {
    this.selectionCallback = null;
    this.clearCallback = null;
    this.isListening = false;
  }

  // Set callback to handle text selection
  onTextSelected(callback) {
    this.selectionCallback = callback;
  }

  // Set callback to handle when selection is cleared
  onSelectionCleared(callback) {
    this.clearCallback = callback;
  }

  // Start listening for text selection
  startListening() {
    if (this.isListening) return;

    document.addEventListener('mouseup', this.handleMouseUp);
    document.addEventListener('keyup', this.handleKeyUp);
    this.isListening = true;
  }

  // Stop listening for text selection
  stopListening() {
    if (!this.isListening) return;

    document.removeEventListener('mouseup', this.handleMouseUp);
    document.removeEventListener('keyup', this.handleKeyUp);
    this.isListening = false;
  }

  // Handle mouse up event (most common text selection method)
  handleMouseUp = () => {
    this.processSelection();
  }

  // Handle keyboard selection (Shift + Arrow keys)
  handleKeyUp = (event) => {
    if (event.key === 'Shift') {
      this.processSelection();
    }
  }

  // Process the current text selection
  processSelection = () => {
    const selection = window.getSelection();
    const selectedText = selection.toString().trim();

    if (selectedText) {
      // Get additional context about the selection
      const context = this.getSelectionContext(selection);

      if (this.selectionCallback) {
        this.selectionCallback({
          content: selectedText,
          context: context,
          timestamp: new Date(),
          range: selection.getRangeAt(0) // Get the range for potential highlighting
        });
      }
    } else {
      // Selection was cleared
      if (this.clearCallback) {
        this.clearCallback();
      }
    }
  }

  // Get context information about the selected text
  getSelectionContext(selection) {
    if (!selection.rangeCount) return null;

    const range = selection.getRangeAt(0);
    const commonAncestor = range.commonAncestorContainer;

    // Try to get the closest section/article/heading for context
    let contextElement = null;

    // Look for closest semantic HTML element
    if (commonAncestor.nodeType === Node.ELEMENT_NODE) {
      contextElement = commonAncestor;
    } else {
      contextElement = commonAncestor.parentElement;
    }

    // Find the closest sectioning content element
    const sectionElement = contextElement.closest('section, article, main, .docs-article, .markdown, .container');

    let context = {
      elementTag: contextElement?.tagName || 'unknown',
      elementClass: contextElement?.className || '',
      elementId: contextElement?.id || '',
      sectionTag: sectionElement?.tagName || 'unknown',
      sectionClass: sectionElement?.className || '',
      sectionId: sectionElement?.id || ''
    };

    return context;
  }

  // Clear the current selection
  clearSelection() {
    if (window.getSelection) {
      window.getSelection().removeAllRanges();
    }

    if (this.clearCallback) {
      this.clearCallback();
    }
  }

  // Get the currently selected text
  getCurrentSelection() {
    const selection = window.getSelection();
    const selectedText = selection.toString().trim();

    if (selectedText) {
      return {
        content: selectedText,
        context: this.getSelectionContext(selection),
        timestamp: new Date()
      };
    }

    return null;
  }
}

// Export a singleton instance
const textSelectionHandler = new TextSelectionHandler();
export default textSelectionHandler;

// Export the class as well for testing purposes if needed
export { TextSelectionHandler };