# /speckit.implement — Implementation Steps (Embedded Book RAG Chatbot MVP)

## Ingestion
- Implement `scripts/ingest_book.py`: load markdown docs, chunk with overlap and metadata, embed via OpenAI, upsert to Qdrant (idempotent), configurable env for keys/collection.
- Verify: log chunk counts, sample payloads, Qdrant collection stats.

## Backend
- Scaffold FastAPI app with routers: `/chat`, `/retrieve`, `/selection`, `/history`.
- Retrieval: Qdrant client wrapper; selection-only filter (overlap check); fallback “insufficient evidence from the selection” when empty.
- Chat: validate input, call retrieval, build prompt (system/assistant + context), invoke ChatKit (OpenAI Agents, 4o mini), stream answer with citations; persist message/selection/citations to Neon.
- Persistence: SQLAlchemy models + migrations for sessions, messages, selections, citations, retrieval_logs; connection via Neon connection string.
- Config: settings for Qdrant endpoint/key, OpenAI key, Neon DSN, chunk/retrieval params, rate limits.

## Frontend
- Add selection capture hook in reader; send selection metadata + `selection_only` flag with question to `/chat`.
- Update chat panel to render streamed responses and citation list; handle “insufficient evidence” states.

## Testing
- Unit: selection-only filter logic; citation injection; prompt includes selection_only flag; validate input schema.
- Integration: `/chat` without selection (book-wide); `/chat` with selection-only returning insufficient evidence when no overlap; `/retrieve` ranking sanity.
- Ingestion smoke: run ingest against sample chapter; assert Qdrant collection populated.

## Ops/Risk
- Add simple rate limiting/middleware; input sanitization for selection spans.
- Graceful errors if Qdrant/Neon unavailable; return clear messages without crash.
- Log retrieval scores, selected chunk ids, selection_only flag, latency.
