# /speckit.plan — Technical Implementation Plan (Embedded Book RAG Chatbot)

## Components
- Frontend (book UI): text selection capture; send {selection_text, chapter, section, page, start_offset?, end_offset?} with question and selection_only flag.
- Backend (FastAPI): routes for chat, retrieve, selection logging, history fetch; orchestrates retrieval + generation.
- Retrieval (Qdrant Cloud Free Tier): vector search over chunked book content with metadata {chapter, section, page, uri, chunk_id, char_start?, char_end?}.
- Orchestration (OpenAI Agents/ChatKit, model: 4o mini): build prompts, inject retrieved chunks/selection subset, stream answers, enforce citations and selection-only policy.
- Persistence (Neon Postgres): tables for sessions, messages, selections, citations, retrieval_logs.
- Ingestion: script to chunk book -> embed (OpenAI embeddings) -> upsert to Qdrant with metadata.

## Backend APIs (FastAPI)
- POST /chat
  - Input: {question, session_id?, selection_only?: bool, selection?: {text, chapter?, section?, page?, start_offset?, end_offset?}}
  - Steps: validate -> retrieve (selection-filtered if flag) -> construct prompt -> stream answer + citations -> persist message/selection/citations -> return stream handle.
- POST /retrieve (internal)
  - Input: {question, top_k?, selection_only?, selection?}
  - Steps: selection filter -> vector search -> rank -> return chunks+scores+metadata.
- POST /selection
  - Store selection metadata tied to session/message.
- GET /history?session_id=
  - Return messages + citations + selections for the session.

## Retrieval Logic
- Default: Qdrant similarity search (top_k, score threshold); re-rank by score + proximity within chapter/section/page.
- Selection-only: filter candidate chunks to those overlapping selection span; optionally include immediate neighbor chunk when explicitly allowed; if no qualifying chunks, respond “insufficient evidence from the selection.”
- Attach metadata for citations (chapter/section/page/uri/offsets).

## Prompting & Generation
- System/assistant prompts from .specify/memory/prompts/*.md.
- Inject retrieved chunks (or selection subset) as context; include selection-only flag in prompt.
- Require inline citations per chunk metadata; refuse or say “insufficient evidence” when support is weak.
- Stream responses via ChatKit; surface partials to UI.

## Ingestion Pipeline
- Parse book content -> chunk by structure/length with overlap.
- Compute embeddings (OpenAI) -> upsert to Qdrant with payload metadata {chapter, section, page, uri, chunk_id, char_start?, char_end?}.
- Maintain idempotent upserts and optional version field.

## Persistence (Neon Postgres)
- sessions(id, created_at)
- messages(id, session_id, role, content, created_at)
- selections(id, session_id, text, chapter, section, page, start_offset, end_offset, created_at)
- citations(id, message_id, chunk_id, chapter, section, page, uri, char_start, char_end, score)
- retrieval_logs(id, session_id, request_id, top_chunks JSON, selection_only bool, created_at)

## Frontend Responsibilities
- Capture highlight spans; package selection metadata.
- Provide toggle for selection_only; default off.
- Display streamed answer with citations and list of source snippets.
- Handle “insufficient evidence” gracefully.

## Error Handling & Degradation
- If Qdrant unavailable: return clear error; optionally fall back to cached recent chunks.
- If Neon unavailable: continue answering but warn that history may not persist.
- Validate inputs: ensure selection offsets and chapter/section/page fields are sane; sanitize text.

## Risk Management
- Enforce strict grounding; refuse when evidence is insufficient or selection-only subset empty.
- Rate-limit per user/session/IP to protect service stability.
- Privacy: store minimal necessary logs; avoid storing API keys or PII in vectors.
- Monitor latency; target <~5s E2E for typical queries.
