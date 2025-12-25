# Physical AI Humanoid Robotics Documentation

This is the documentation site for the Physical AI Humanoid Robotics project, built with Docusaurus.

## RAG Knowledge Ingestion and Retrieval Pipeline

This repository includes a comprehensive RAG (Retrieval-Augmented Generation) Knowledge Ingestion and Retrieval Pipeline that can:

### Ingestion Pipeline
- Crawl Docusaurus documentation sites
- Extract clean text content
- Chunk content using semantic-aware strategies
- Generate embeddings using Cohere API
- Store embeddings in Qdrant Cloud for similarity search
- Validate the ingestion process

### Retrieval Pipeline
- Perform semantic search against stored content using vector similarity
- Support metadata-based filtering (URL, section, etc.)
- Provide configurable top-k retrieval
- Include validation framework for accuracy testing
- Offer comprehensive logging for retrieval metrics

### Setup

1. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables in `.env`:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys and configuration
   ```

### Usage

#### 1. Ingest Documentation Content
```bash
# Run the ingestion pipeline
python run_local_pipeline.py ingest --site-url "https://docusaurus.io/docs"

# With custom chunking parameters
python run_local_pipeline.py ingest --site-url "https://example.com" --chunk-size 800 --chunk-overlap 100
```

#### 2. Retrieve Semantically Similar Content
```bash
# Basic retrieval
python run_local_pipeline.py retrieve "What are the main principles of AI?"

# With custom top-k parameter
python run_local_pipeline.py retrieve "machine learning concepts" --top-k 10

# With metadata filtering
python run_local_pipeline.py retrieve "neural networks" --filter url=https://example.com/page --top-k 5

# Output in JSON format
python run_local_pipeline.py retrieve "AI ethics" --format json
```

#### 3. Validate Retrieval System
```bash
# Run accuracy validation
python run_local_pipeline.py validate

# Run validation with benchmarks
python run_local_pipeline.py validate --benchmark

# With custom top-k for validation
python run_local_pipeline.py validate --top-k 3
```

#### 4. Run in Local Mode (without external APIs)
```bash
# Set LOCAL_MODE=true in your environment or .env file
export LOCAL_MODE=true

# The system will use mock implementations for testing
python run_local_pipeline.py
```

## Deployment to Vercel

This project is configured for deployment on Vercel. Follow these steps to deploy:

### 1. Prepare Your Repository
1. Push this code to a GitHub repository
2. Make sure all changes are committed

### 2. Deploy via Vercel Dashboard
1. Go to [Vercel](https://vercel.com)
2. Sign in with your GitHub account
3. Click "New Project" and select your GitHub repository
4. Vercel will automatically detect this is a Docusaurus project
5. Use the following build settings:
   - Build Command: `npm run build` or `npm run vercel-build`
   - Output Directory: `build`
   - Install Command: `npm install`
6. Click "Deploy"

### 3. Deploy via Vercel CLI (Alternative)
1. Install Vercel CLI: `npm install -g vercel`
2. Login: `vercel login`
3. Deploy: `vercel`

### Configuration Notes
- The site is configured to work at the root path (`/`) for Vercel deployment
- Custom domain can be added in the Vercel dashboard after deployment
- Environment variables can be configured in the Vercel dashboard if needed

## Local Development

```bash
npm install
npm start
```

## Building for Production

```bash
npm run build
```

The built files will be in the `build` directory and are ready for deployment.

## Customization

- Edit `docusaurus.config.js` to change site configuration
- Edit CSS in `src/css/custom.css` to customize styling
- Add documentation in the `docs` directory
- Customize pages in the `src/pages` directory