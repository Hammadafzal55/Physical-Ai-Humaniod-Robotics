import os
import logging
from typing import List, Dict, Any, Optional
import uuid
import re
from functools import wraps
from configparser import ConfigParser
import datetime # Added
from playwright.sync_api import sync_playwright, Error # Added Error
import concurrent.futures # Added
import sys # Added

from pydantic import BaseModel, Field

from dotenv import load_dotenv
from trafilatura.sitemaps import sitemap_search

from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

import cohere

from cohere.core.api_error import ApiError

import qdrant_client

from qdrant_client.http.models import Distance, VectorParams, PointStruct, UpdateStatus

from qdrant_client.http.exceptions import UnexpectedResponse

# Load environment variables
load_dotenv()

# --- Configuration ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
if not COHERE_API_KEY:
    logger.error("COHERE_API_KEY environment variable not set.")
    # For a script, we should exit if key components are missing
    sys.exit(1)

QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not QDRANT_HOST:
    logger.error("QDRANT_HOST environment variable not set.")
    sys.exit(1)

try:
    cohere_client = cohere.Client(COHERE_API_KEY)
    qdrant_client_instance = qdrant_client.QdrantClient(url=QDRANT_HOST, api_key=QDRANT_API_KEY, timeout=60)
except Exception as e:
    logger.error(f"Failed to initialize API clients: {e}")
    sys.exit(1)

# --- Pydantic Models for Internal Data Structures ---
class TextChunkMetadata(BaseModel):
    source_url: str
    chapter_title: Optional[str] = None
    module_name: Optional[str] = None
    timestamp: Optional[str] = None # ISO 8601 format

class TextChunk(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    content: str
    metadata: TextChunkMetadata

# --- Utility Functions ---

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=4, max=10),
    retry=retry_if_exception_type((Exception, Error)), # Catch playwright-specific errors too
    reraise=True
)
def extract_text_from_url(url: str) -> Optional[Dict[str, Any]]:
    """
    Fetches content from a given URL using Playwright for robust rendering,
    extracts clean text, and captures metadata.
    Includes retry logic for network requests.
    """
    logger.info(f"Attempting to extract content from {url} using Playwright.")
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()
        page.set_default_timeout(20000) # Set default timeout for Playwright operations
        try:
            page.goto(url, wait_until="domcontentloaded", timeout=30000) # 30 seconds timeout
            
            text = ""
            # Prioritize 'article' tag for main content
            main_content_element = page.query_selector("article")
            if main_content_element:
                text = main_content_element.inner_text()
            
            # Fallback to body if no article or empty text
            if not text:
                logger.warning(f"No text extracted from 'article' tag for {url}. Attempting full page text.")
                text = page.body().inner_text()

            if not text:
                logger.error(f"Playwright extracted empty content from {url}.")
                return None

            title = page.title() # Get page title
            
            # Refined metadata extraction
            chapter_title = title.replace(" | Physical AI Humanoid Robotics", "").strip() if title else "Unknown Chapter"
            module_name = "Unknown Module"
            
            path_parts = [part for part in url.split('/') if part]
            if "docs" in path_parts:
                try:
                    docs_index = path_parts.index("docs")
                    if len(path_parts) > docs_index + 1:
                        module_slug = path_parts[docs_index + 1]
                        module_name = module_slug.replace('-', ' ').title() # Basic slug to name conversion
                except ValueError:
                    pass # "docs" not in path_parts or other indexing issues
            
            # Set timestamp directly
            timestamp = datetime.datetime.now(datetime.timezone.utc).isoformat() # ISO 8601 timestamp

            logger.info(f"Successfully extracted text from {url} using Playwright.")
            return {
                "text": text,
                "metadata": {
                    "source_url": url,
                    "chapter_title": chapter_title,
                    "module_name": module_name,
                    "timestamp": timestamp,
                }
            }
        except Exception as e:
            logger.error(f"Playwright failed to extract content from {url}: {e}")
            return None
        finally:
            browser.close()

def chunk_text(text: str, max_chunk_chars: int = 1500, overlap_chars: int = 100) -> List[str]:
    """
    Splits a long text into fixed-size chunks with overlap.
    """
    chunks = []
    current_position = 0
    while current_position < len(text):
        end_position = min(current_position + max_chunk_chars, len(text))
        chunk = text[current_position:end_position]
        chunks.append(chunk.strip())
        current_position += (max_chunk_chars - overlap_chars)
        if current_position < 0:
            current_position = 0 
    
    if len(chunks) > 1 and chunks[-1] == chunks[-2]:
        chunks.pop()

    logger.info(f"Chunked text into {len(chunks)} chunks.")
    return chunks

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=4, max=10),
    retry=retry_if_exception_type(ApiError),
    reraise=True
)
def embed(texts: List[str]) -> List[List[float]]:
    """
    Generates embeddings for a list of text chunks using Cohere embed-english-v3.0.
    """
    if not texts:
        return []

    logger.info(f"Generating embeddings for {len(texts)} texts using Cohere...")
    response = cohere_client.embed(
        texts=texts,
        model="embed-english-v3.0",
        input_type="search_document"
    )
    embeddings = response.embeddings
    logger.info(f"Successfully generated {len(embeddings)} embeddings.")
    return embeddings

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=4, max=10),
    retry=retry_if_exception_type(UnexpectedResponse),
    reraise=True
)
def create_collection(
    collection_name: str,
    vector_size: int = 1024, # Cohere embed-english-v3.0 default
    distance_metric: Distance = Distance.COSINE
):
    """
    Initializes a Qdrant collection.
    """
    logger.info(f"Checking for collection '{collection_name}' in Qdrant...")
    if not qdrant_client_instance.collection_exists(collection_name=collection_name):
        logger.info(f"Collection '{collection_name}' not found. Creating...")
        qdrant_client_instance.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=vector_size, distance=distance_metric),
        )
        qdrant_client_instance.create_payload_index(
            collection_name=collection_name, field_name="source_url", field_type="keyword"
        )
        qdrant_client_instance.create_payload_index(
            collection_name=collection_name, field_name="chapter_title", field_type="keyword"
        )
        qdrant_client_instance.create_payload_index(
            collection_name=collection_name, field_name="module_name", field_type="keyword"
        )
        logger.info(f"Collection '{collection_name}' created and indexed.")
    else:
        logger.info(f"Collection '{collection_name}' already exists.")

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=4, max=10),
    retry=retry_if_exception_type(UnexpectedResponse),
    reraise=True
)
def save_chunk_to_qdrant(collection_name: str, text_chunk: TextChunk, embedding: List[float]):       
    """
    Stores a text chunk and its embedding in the specified Qdrant collection.
    """
    payload = {
        "content": text_chunk.content,
        "source_url": text_chunk.metadata.source_url,
        "chapter_title": text_chunk.metadata.chapter_title,
        "module_name": text_chunk.metadata.module_name,
        "timestamp": text_chunk.metadata.timestamp,
    }

    point = PointStruct(
        id=text_chunk.id,
        vector=embedding,
        payload=payload
    )

    response = qdrant_client_instance.upsert(
        collection_name=collection_name,
        points=[point],
        wait=True,
    )
    if response.status != UpdateStatus.COMPLETED:
        logger.error(f"Failed to save chunk {text_chunk.id} to Qdrant: {response.status}")
        raise Exception(f"Failed to upsert to Qdrant: {response.status}")

def ingest_book(collection_name: str, textbook_urls: List[str]):
    """
    Orchestrates the entire RAG pipeline for data ingestion.
    """
    if not textbook_urls:
        logger.warning("No textbook URLs provided for ingestion.")
        return

    vector_size = 1024 # Cohere embed-english-v3.0 default
    create_collection(collection_name, vector_size)

    MAX_WORKERS = 4
    with concurrent.futures.ThreadPoolExecutor(max_workers=MAX_WORKERS) as executor:
        future_to_url = {executor.submit(_process_single_url, url, collection_name, vector_size): url for url in textbook_urls}
        for future in concurrent.futures.as_completed(future_to_url):
            url = future_to_url[future]
            try:
                future.result() 
                logger.info(f"Finished processing and ingesting from URL: {url}")
            except Exception as e:
                logger.error(f"Error processing URL {url} in parallel: {e}")

def _process_single_url(url: str, collection_name: str, vector_size: int):
    """
    Helper function to process a single URL for parallel execution.
    """
    logger.info(f"Processing URL: {url}")
    if not re.match(r"https?://(?:[-\w.]|(?:%[\da-fA-F]{2}))+", url):
        logger.error(f"Invalid URL format: {url}. Skipping.")
        return

    extracted_data = extract_text_from_url(url)
    if not extracted_data:
        logger.warning(f"Could not extract data for URL: {url}. Skipping ingestion.")
        return
    
    text = extracted_data["text"]
    metadata = extracted_data["metadata"]

    chunks = chunk_text(text)
    
    batch_size = 96
    for i in range(0, len(chunks), batch_size):
        chunk_batch = chunks[i : i + batch_size]
        embeddings_batch = embed(chunk_batch)

        for j, chunk_content in enumerate(chunk_batch):
            metadata_instance = TextChunkMetadata(
                source_url=metadata.get("source_url"),
                chapter_title=metadata.get("chapter_title"),
                module_name=metadata.get("module_name"),
                timestamp=metadata.get("timestamp"),
            )
            text_chunk_instance = TextChunk(
                content=chunk_content,
                metadata=metadata_instance
            )
            save_chunk_to_qdrant(collection_name, text_chunk_instance, embeddings_batch[j])

def get_textbook_chapter_urls() -> List[str]:
    """
    Fetches chapter URLs from a sitemap URL in the TEXTBOOK_URLS env var.
    """
    sitemap_url = os.getenv("TEXTBOOK_URLS")
    if not sitemap_url:
        logger.error("TEXTBOOK_URLS environment variable not set.")
        return []
    
    urls = sitemap_search(sitemap_url)
    
    if not urls:
        logger.warning(f"No URLs found in the sitemap: {sitemap_url}")
        return []

    urls = [url for url in urls if not url.endswith('.xml')]
    logger.info(f"Loaded {len(urls)} textbook URLs from sitemap: {sitemap_url}")
    return urls

def verify_qdrant_data(collection_name: str, query_text: str = "What are the main components of ROS 2?", limit: int = 3):
    """
    Queries Qdrant to verify ingested data by performing a sample search.
    """
    try:
        logger.info(f"Verifying Qdrant data in collection '{collection_name}' with query: '{query_text}'")
        
        query_embedding_response = cohere_client.embed(
            texts=[query_text], model="embed-english-v3.0", input_type="search_query"
        )
        query_embedding = query_embedding_response.embeddings[0]

        search_result = qdrant_client_instance.query_points(
            collection_name=collection_name, query=query_embedding, limit=limit, with_payload=True
        )

        if not search_result.points:
            logger.warning(f"No results found for query '{query_text}' in collection '{collection_name}'.")
            return

        logger.info(f"Top {limit} results for query: '{query_text}'")
        for hit in search_result.points:
            logger.info(f"  Score: {hit.score}")
            logger.info(f"  Text: {hit.payload['content'][:200]}...")
            logger.info(f"  Source: {hit.payload['source_url']}\n")
        
        logger.info("Qdrant data verification completed.")
        return True

    except Exception as e:
        logger.error(f"Error during Qdrant data verification: {e}", exc_info=True)
        return False

# --- Main execution logic for direct script execution ---
if __name__ == "__main__":
    logger.info("Starting RAG Pipeline ingestion process.")

    COLLECTION_NAME = "Physical Ai Book"
    
    TEXTBOOK_URLS = get_textbook_chapter_urls()

    if not TEXTBOOK_URLS:
        logger.error("No textbook URLs found. Please ensure TEXTBOOK_URLS environment variable is set and valid.")
    else:
        try:
            ingest_book(COLLECTION_NAME, TEXTBOOK_URLS)
            logger.info("RAG Pipeline ingestion completed successfully.")

            logger.info("Starting Qdrant data verification...")
            if verify_qdrant_data(COLLECTION_NAME):
                logger.info("Qdrant data verification successful.")
            else:
                logger.warning("Qdrant data verification failed or returned no results.")
        except Exception as e:
            logger.critical(f"A critical error occurred during the RAG pipeline ingestion: {e}", exc_info=True)
            sys.exit(1)
