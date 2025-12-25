# RAG Knowledge Ingestion Pipeline - Example Usage

## Overview

The RAG Knowledge Ingestion Pipeline is a CLI tool that:
- Crawls Docusaurus documentation sites
- Extracts clean text content
- Chunks content using semantic-aware strategies
- Generates embeddings using Cohere API
- Stores embeddings in Qdrant Cloud for similarity search
- Validates the ingestion process

## Setup

1. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables in `.env`:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys and configuration
   ```

## Configuration

The pipeline uses the following environment variables:

- `COHERE_API_KEY`: Your Cohere API key for generating embeddings
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `DOCUSAURUS_SITE_URL`: The URL of the Docusaurus site to crawl
- `CHUNK_SIZE`: Size of text chunks (default: 1000)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 200)
- `LOG_LEVEL`: Logging level (default: INFO)

## Usage

### Basic Usage

Run the pipeline with default settings:

```bash
python -m src.rag_pipeline.cli.main --site-url "https://your-docusaurus-site.com"
```

### Advanced Usage

Run the pipeline with custom chunking parameters:

```bash
python -m src.rag_pipeline.cli.main \
  --site-url "https://your-docusaurus-site.com" \
  --chunk-size 1500 \
  --chunk-overlap 300
```

### With Validation

Run the pipeline with post-ingestion validation:

```bash
python -m src.rag_pipeline.cli.main \
  --site-url "https://your-docusaurus-site.com" \
  --validate
```

## Pipeline Components

The pipeline consists of several modules:

- `crawler/`: Handles crawling Docusaurus sites and extracting content
- `chunker/`: Manages content chunking with semantic-aware strategies
- `embedder/`: Generates embeddings using Cohere API
- `storage/`: Stores embeddings in Qdrant Cloud
- `validators/`: Validates the ingestion process
- `cli/`: Main command-line interface

## Architecture

The pipeline follows a modular architecture:

1. **Crawling**: SiteCrawler crawls the Docusaurus site and extracts HTML content
2. **Extraction**: ContentExtractor cleans the HTML and extracts text content
3. **Chunking**: ContentChunker splits content into semantic chunks
4. **Embedding**: CohereEmbedder generates vector embeddings for each chunk
5. **Storage**: QdrantStorage stores embeddings and metadata in Qdrant Cloud
6. **Validation**: IngestionValidator verifies the ingestion process

## Error Handling

The pipeline includes comprehensive error handling:

- CrawlerError: Issues with crawling the site
- ContentExtractionError: Problems extracting content from HTML
- ChunkingError: Issues with content chunking
- EmbeddingError: Problems with embedding generation
- StorageError: Issues storing data in Qdrant
- ValidationError: Problems with validation

## Validation

The pipeline includes validation functionality that:

- Runs sample similarity queries against the stored embeddings
- Validates data integrity in Qdrant
- Checks metadata completeness
- Provides overall validation status

## Troubleshooting

- If you get API key errors, ensure your `.env` file is properly configured
- If crawling is slow, consider increasing the delay between requests in the SiteCrawler
- If you run into rate limits with Cohere, consider reducing the batch size in the embedder
- Check the logs for detailed error messages

## Example Output

When the pipeline runs successfully, you'll see:

```
Starting RAG Knowledge Ingestion Pipeline...
Crawling site: https://your-site.com
Crawled 25 pages successfully
Extracting clean content from crawled pages...
Extracted content from 25 pages
Chunking content (size: 1000, overlap: 200)...
Created 142 content chunks
Generating embeddings for chunks...
Generated embeddings for 142 chunks
Storing embeddings in Qdrant Cloud...
Successfully stored all chunks in Qdrant Cloud
RAG Knowledge Ingestion Pipeline completed successfully!
```