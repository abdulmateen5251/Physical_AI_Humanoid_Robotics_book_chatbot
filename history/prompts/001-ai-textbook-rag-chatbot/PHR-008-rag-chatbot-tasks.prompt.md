# /speckit.tasks — Implementation Tasks (Embedded Book RAG Chatbot)

## Data Ingestion
- Write `scripts/ingest_book.py` to parse book markdown, chunk by structure/length with overlap, add metadata (chapter, section, page, uri, chunk_id, char_start/char_end).
- Generate embeddings via OpenAI; upsert chunks into Qdrant Cloud Free Tier with payload metadata and idempotent upserts.
- Add config for Qdrant endpoint/key, OpenAI key, chunk sizes, overlap, and collection name.

## Backend (FastAPI)
- Create FastAPI app with routes: POST `/chat`, POST `/retrieve`, POST `/selection`, GET `/history`.
- Implement retrieval service: Qdrant similarity search with selection-only filter (overlap on selection span; neighbor optional) and score thresholding.
- Implement chat handler: validate inputs, run retrieval, build prompt (system + assistant + retrieved chunks/selection), call ChatKit (OpenAI Agents, model 4o mini), stream answer, attach citations, persist message/selection/citations to Neon.
- Persist tables: sessions, messages, selections, citations, retrieval_logs in Neon Postgres; add migrations.
- Add rate limiting and input validation (selection spans, chapter/section/page sanity checks).

## Frontend
- Add selection capture in reader: collect highlighted text + chapter/section/page + offsets; toggle `selection_only`.
- Extend chat UI to send question + selection metadata to `/chat`, display streamed answer and citations.
- Handle “insufficient evidence from the selection” gracefully in UI.

## Testing
- Unit tests: selection-only filter logic; citation presence enforcement; prompt construction includes selection_only flag.
- Integration tests: end-to-end `/chat` with and without selection; ensure refusal when selection has no supporting chunks.
- Smoke test ingestion: chunks count > 0, embeddings present, Qdrant upsert succeeds.

## Risk Management
- Validate empty/invalid selections; short-circuit with friendly error.
- Fallback message: “insufficient evidence” when selection filter yields no chunks.
- Graceful degradation if Qdrant/Neon unavailable; return clear errors and avoid crashes.
