# Embedded RAG Chatbot Constitution

- Scope: Book-only RAG chatbot embedded in the published book; orchestrated via OpenAI Agents/ChatKit, served by FastAPI.

## Quality
- Ground every answer in retrieved book text; include citations (chapter/section/page/URI).
- Selection-only mode: constrain evidence to the highlighted text (and only neighbor lines if explicitly enabled); if insufficient, reply “insufficient evidence from the selected text.”
- Non-selection mode: use top retrieved chunks; refuse or defer if evidence is weak.
- Keep answers concise, factual, and free of speculation.

## Safety
- If the book (or selection) lacks sufficient evidence, state inability to answer from the book; do not invent or use external knowledge.
- Avoid unsafe content; refuse disallowed requests per platform policy.

## Architecture
- Retrieval: Qdrant Cloud (Free Tier) for vector search over chunked book content.
- Orchestration: OpenAI Agents/ChatKit (model: 4o mini) for prompting and streaming.
- Backend: FastAPI service layer for chat/retrieval endpoints.
- Persistence: Neon Serverless Postgres for sessions, messages, selections, citations.

## Data & Ingestion
- Chunk book by structure/length; store metadata: {chapter, section, page, URI, chunk_id}.
- Embed chunks with OpenAI embeddings; upsert to Qdrant with metadata payload.
- Maintain chunk overlaps to preserve context boundaries; keep chunk sizes latency-friendly.

## Retrieval Logic
- Default: similarity search in Qdrant (top-k), then rank by score and proximity within the same chapter/section/page.
- Selection-only toggle: filter to the highlighted text; optionally include immediate neighbor lines only when explicitly enabled.
- If no qualifying chunks, return “insufficient evidence.”

## UX
- Inline chat in the book UI with a toggle for “selected text only.”
- Always show citations with chapter/section/page/URI alongside answers.
- Present concise answers; avoid long-form unless explicitly requested.

## Persistence
- Store sessions, messages, selections, and citations in Neon Postgres.
- Optionally mirror session transcripts to repository history files for auditability (when allowed and privacy-safe).

## Evaluation
- Validate: grounding correctness, citation presence, selection-only enforcement, refusal on weak evidence.
- Performance: keep typical end-to-end latency acceptable for reading (target < ~5s).
- Resilience: graceful degradation if Qdrant or Neon is unavailable; surface clear errors.

## Risk Management
- Enforce strict grounding; refuse when evidence is insufficient.
- Rate-limit requests per user/session/IP to protect service stability.
- Privacy: minimize logged data; protect user selections and chat history; avoid sharing data with third parties beyond required providers.
- Observability: log retrieval scores, selected chunks, selection-only flag, and latencies for debugging and QA.
