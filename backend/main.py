import os
import logging
from typing import List, Dict, Any
import uuid
import re
from functools import wraps

from dotenv import load_dotenv
from requests_html import HTMLSession
from bs4 import BeautifulSoup
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type
import cohere
from cohere.error import CohereAPIError
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
    # Exit or raise an exception, depending on desired behavior
    # For now, we'll just log and let it fail later if used
    # exit(1)

QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not QDRANT_HOST:
    logger.error("QDRANT_HOST environment variable not set.")
    # exit(1)

cohere_client = cohere.Client(COHERE_API_KEY)
qdrant_client_instance = qdrant_client.QdrantClient(host=QDRANT_HOST, api_key=QDRANT_API_KEY)


# --- Utility Functions ---

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=4, max=10),
    retry=retry_if_exception_type(Exception), # Catch broader exceptions for now
    reraise=True
)
def extract_text_from_url(url: str) -> Dict[str, Any]:
    """
    Fetches HTML content from a given URL, parses it, and extracts clean text along with metadata.
    Includes retry logic for network requests.
    """
    session = HTMLSession()
    response = session.get(url)
    response.raise_for_status()  # Raise HTTPError for bad responses (4xx or 5xx)

    # Use BeautifulSoup for parsing
    soup = BeautifulSoup(response.text, 'html.parser')

    # Remove script, style, and other non-text elements
    for script_or_style in soup(['script', 'style', 'header', 'footer', 'nav', '.navbar', '.sidebar', '.table-of-contents']):
        script_or_style.decompose()

    # Extract title and other potential metadata
    title = soup.find('h1').get_text(strip=True) if soup.find('h1') else url
    # Basic attempt to get module/chapter names from URL path if Docusaurus specific structure is assumed
    path_parts = [part for part in url.split('/') if part]
    module_name = "Unknown Module"
    chapter_title = "Unknown Chapter"

    # Example: https://physical-ai-humaniod-robotics.vercel.app/docs/module-1-ros-2/introduction-to-physical-ai
    if "docs" in path_parts:
        try:
            docs_index = path_parts.index("docs")
            if len(path_parts) > docs_index + 1:
                module_slug = path_parts[docs_index + 1]
                module_name = module_slug.replace('-', ' ').title() # Basic slug to name conversion
            if len(path_parts) > docs_index + 2:
                chapter_slug = path_parts[docs_index + 2]
                chapter_title = chapter_slug.replace('-', ' ').title().replace('.Md', '') # Remove .md if present and title case
        except ValueError:
            pass # "docs" not in path_parts or other indexing issues

    # Extract clean text
    main_content = soup.find('article') # Docusaurus content is often within an <article> tag
    if not main_content:
        main_content = soup.find('main')
    if not main_content:
        main_content = soup.body
        
    text = main_content.get_text(separator=' ', strip=True)
    # Clean up multiple spaces and newlines
    text = re.sub(r'\s+', ' ', text).strip()

    logger.info(f"Successfully extracted text from {url}")
    return {
        "text": text,
        "metadata": {
            "source_url": url,
            "chapter_title": chapter_title,
            "module_name": module_name,
            # Add more metadata as needed
        }
    }

def chunk_text(text: str, max_chunk_chars: int = 1500, overlap_chars: int = 100) -> List[str]:
    """
    Splits a long text into semantically meaningful chunks with overlap,
    adhering to a maximum character limit (proxy for token limit).
    This simple chunking prioritizes paragraph boundaries.
    """
    chunks = []
    paragraphs = text.split('\n\n') # Split by paragraph

    current_chunk = ""
    for paragraph in paragraphs:
        if len(current_chunk) + len(paragraph) + 2 < max_chunk_chars: # +2 for potential newline
            current_chunk += (paragraph + "\n\n")
        else:
            if current_chunk:
                chunks.append(current_chunk.strip())
            current_chunk = paragraph + "\n\n"

            # If a single paragraph is too large, split it further by sentences
            while len(current_chunk) > max_chunk_chars:
                sentences = re.split(r'(?<=[.!?])\s+', current_chunk)
                sub_chunk = ""
                for sentence in sentences:
                    if len(sub_chunk) + len(sentence) + 1 < max_chunk_chars:
                        sub_chunk += (sentence + " ")
                    else:
                        if sub_chunk:
                            chunks.append(sub_chunk.strip())
                            # Add overlap
                            overlap_start = max(0, len(sub_chunk) - overlap_chars)
                            current_chunk = sub_chunk[overlap_start:] + current_chunk # Start new chunk with overlap
                            break
                        else: # Single sentence larger than max_chunk_chars
                            chunks.append(current_chunk[:max_chunk_chars].strip())
                            current_chunk = current_chunk[max_chunk_chars - overlap_chars:] # Overlap for very long sentences
                            break
                else: # If all sentences fit
                    if sub_chunk:
                        chunks.append(sub_chunk.strip())
                    current_chunk = "" # No remaining text

    if current_chunk:
        chunks.append(current_chunk.strip())
    
    # Final pass to ensure all chunks respect max_chunk_chars and add explicit overlap
    final_chunks = []
    for i, chunk in enumerate(chunks):
        if len(chunk) > max_chunk_chars: # Re-chunk if paragraph/sentence logic failed for very long ones
            for j in range(0, len(chunk), max_chunk_chars - overlap_chars):
                sub_chunk = chunk[j : j + max_chunk_chars]
                final_chunks.append(sub_chunk.strip())
        else:
            final_chunks.append(chunk)

    logger.info(f"Chunked text into {len(final_chunks)} chunks.")
    return final_chunks

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=4, max=10),
    retry=retry_if_exception_type(CohereAPIError),
    reraise=True
)
def embed(texts: List[str]) -> List[List[float]]:
    """
    Generates embeddings for a list of text chunks using Cohere embed-english-v3.0.
    Includes retry logic for Cohere API calls.
    """
    if not texts:
        return []
    
    logger.info(f"Generating embeddings for {len(texts)} texts using Cohere...")
    response = cohere_client.embed(
        texts=texts,
        model="embed-english-v3.0",
        input_type="search_document" # Or "search_query" for query embeddings
    )
    embeddings = response.embeddings
    logger.info(f"Successfully generated {len(embeddings)} embeddings.")
    return embeddings

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=4, max=10),
    retry=retry_if_exception_type(UnexpectedResponse), # Qdrant API errors
    reraise=True
)
def create_collection(
    collection_name: str, 
    vector_size: int = 1024, # Cohere embed-english-v3.0 default
    distance_metric: Distance = Distance.COSINE
):
    """
    Initializes a Qdrant collection with specified parameters and payload schema.
    """
    logger.info(f"Checking for collection '{collection_name}' in Qdrant...")
    if not qdrant_client_instance.collection_exists(collection_name=collection_name):
        logger.info(f"Collection '{collection_name}' not found. Creating...")
        qdrant_client_instance.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=vector_size, distance=distance_metric),
        )
        # Define payload schema for better filtering and indexing
        qdrant_client_instance.update_collection(
            collection_name=collection_name,
            optimizer_config=qdrant_client_instance.get_collection(collection_name).optimizer_config,
            # Add payload_schema definitions for metadata fields here if Qdrant version supports it
            # For newer Qdrant versions, index creation is typically separate
        )
        # Ensure fields are indexed for filtering
        qdrant_client_instance.create_payload_index(
            collection_name=collection_name,
            field_name="source_url",
            field_type="keyword"
        )
        qdrant_client_instance.create_payload_index(
            collection_name=collection_name,
            field_name="chapter_title",
            field_type="keyword"
        )
        qdrant_client_instance.create_payload_index(
            collection_name=collection_name,
            field_name="module_name",
            field_type="keyword"
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
def save_chunk_to_qdrant(collection_name: str, text_chunk: Dict[str, Any], embedding: List[float]):
    """
    Stores a text chunk and its embedding in the specified Qdrant collection.
    """
    point_id = str(uuid.uuid4()) # Generate a unique ID for the point
    
    # Ensure all metadata fields are present, even if empty
    payload = {
        "content": text_chunk["text"],
        "source_url": text_chunk["metadata"].get("source_url", "unknown"),
        "chapter_title": text_chunk["metadata"].get("chapter_title", "unknown"),
        "module_name": text_chunk["metadata"].get("module_name", "unknown"),
        # Add other metadata fields if available
    }

    point = PointStruct(
        id=point_id,
        vector=embedding,
        payload=payload
    )

    response = qdrant_client_instance.upsert(
        collection_name=collection_name,
        points=[point],
        wait=True,
    )
    if response.status == UpdateStatus.COMPLETED:
        logger.debug(f"Successfully saved chunk {point_id} to Qdrant.")
    else:
        logger.error(f"Failed to save chunk {point_id} to Qdrant: {response.status}")
        raise Exception(f"Failed to upsert to Qdrant: {response.status}")

def ingest_book(collection_name: str, textbook_urls: List[str]):
    """
    Orchestrates the entire RAG pipeline:
    1. Fetches content from URLs.
    2. Extracts clean text and metadata.
    3. Chunks text into semantically meaningful pieces.
    4. Generates embeddings for each chunk.
    5. Stores embeddings and metadata in Qdrant.
    """
    if not textbook_urls:
        logger.warning("No textbook URLs provided for ingestion.")
        return

    vector_size = 1024 # Cohere embed-english-v3.0 default
    create_collection(collection_name, vector_size)

    for url in textbook_urls:
        logger.info(f"Processing URL: {url}")
        try:
            extracted_data = extract_text_from_url(url)
            text = extracted_data["text"]
            metadata = extracted_data["metadata"]

            chunks = chunk_text(text)
            
            # Process chunks in batches to optimize Cohere API calls
            batch_size = 96 # Cohere API limit is 96 texts per request
            for i in range(0, len(chunks), batch_size):
                chunk_batch = chunks[i : i + batch_size]
                embeddings_batch = embed(chunk_batch)

                for j, chunk in enumerate(chunk_batch):
                    full_metadata = metadata.copy()
                    full_metadata["content"] = chunk # Store original chunk text in payload
                    full_metadata["chunk_number"] = i + j
                    
                    # Create a TextChunk-like dictionary for save_chunk_to_qdrant
                    text_chunk_payload = {
                        "text": chunk,
                        "metadata": full_metadata
                    }
                    save_chunk_to_qdrant(collection_name, text_chunk_payload, embeddings_batch[j])
            logger.info(f"Finished processing and ingesting from URL: {url}")

        except Exception as e:
            logger.error(f"Error processing URL {url}: {e}")

def get_textbook_chapter_urls() -> List[str]:
    """
    Fetches a list of deployed Docusaurus chapter URLs from the TEXTBOOK_URLS environment variable.
    """
    textbook_urls_str = os.getenv("TEXTBOOK_URLS")
    if not textbook_urls_str:
        logger.error("TEXTBOOK_URLS environment variable not set.")
        return []
    
    urls = [url.strip() for url in textbook_urls_str.split(',') if url.strip()]
    logger.info(f"Loaded {len(urls)} textbook URLs from environment variables.")
    return urls

# --- Main execution logic (will be implemented in T016) ---
if __name__ == "__main__":
    logger.info("Starting RAG Pipeline ingestion.")
    
    COLLECTION_NAME = "textbook_embeddings"
    TEXTBOOK_URLS = get_textbook_chapter_urls()

    if not TEXTBOOK_URLS:
        logger.error("No textbook URLs found. Please set TEXTBOOK_URLS environment variable.")
    else:
        try:
            ingest_book(COLLECTION_NAME, TEXTBOOK_URLS)
            logger.info("RAG Pipeline ingestion completed successfully.")
            
            # Verify data in Qdrant after ingestion
            logger.info("Starting Qdrant data verification.")
            if verify_qdrant_data(COLLECTION_NAME):
                logger.info("Qdrant data verification successful.")
            else:
                logger.warning("Qdrant data verification failed or returned no results.")
        except Exception as e:
            logger.critical(f"RAG Pipeline ingestion failed: {e}", exc_info=True)

def verify_qdrant_data(collection_name: str, query_text: str = "What are the main components of ROS 2?", limit: int = 3):
    """
    Queries Qdrant to verify ingested data by performing a sample search.
    Adapts the example from quickstart.md (SC-004).
    """
    try:
        if not cohere_client or not COHERE_API_KEY:
            logger.error("Cohere client not initialized. Cannot verify data.")
            return

        logger.info(f"Verifying Qdrant data in collection '{collection_name}' with query: '{query_text}'")
        
        # Generate embedding for the query
        query_embedding_response = cohere_client.embed(
            texts=[query_text],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_embedding = query_embedding_response.embeddings[0]

        # Search Qdrant
        search_result = qdrant_client_instance.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True
        )

        if not search_result:
            logger.warning(f"No results found for query '{query_text}' in collection '{collection_name}'.")
            return

        logger.info(f"Top {limit} results for query: '{query_text}'")
        for hit in search_result:
            logger.info(f"  Score: {hit.score}")
            logger.info(f"  Text: {hit.payload['content'][:200]}...") # Print first 200 chars
            logger.info(f"  Source: {hit.payload['source_url']}")
            logger.info(f"  Chapter: {hit.payload['chapter_title']}\n")
        
        logger.info("Qdrant data verification completed.")
        return True

    except Exception as e:
        logger.error(f"Error during Qdrant data verification: {e}", exc_info=True)
        return False



