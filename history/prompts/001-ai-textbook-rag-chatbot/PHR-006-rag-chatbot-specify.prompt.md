# /speckit.specify — Embedded RAG Chatbot (Book)

## Goals
- Provide grounded Q&A over the book via OpenAI Agents/ChatKit + FastAPI backend.
- Support selection-only answers: restrict evidence to highlighted text (and immediate neighbors only when explicitly enabled).
- Persist sessions, messages, selections, and citations in Neon Postgres.
- Use Qdrant Cloud Free Tier for retrieval over chunked book content.

## Behaviors
- Book-wide Q&A: retrieve top-k chunks from Qdrant (chapter/section/page metadata), answer with citations.
- Selection-only Q&A: when selection is provided and flag set, filter retrieval to chunks overlapping the selection; answer only from that subset; otherwise say “insufficient evidence from the selection.”
- Stream answers via OpenAI Agents/ChatKit; keep responses concise and cited.
- Refuse to answer outside-book questions or when evidence is weak.

## Backend Endpoints (FastAPI)
- POST /chat: {question, selection? {text, chapter, section, page}, selection_only?: bool, session_id}: returns streamed answer + citations.
- POST /retrieve: internal retrieval; supports selection_only filter; returns ranked chunks with scores and metadata.
- POST /selection: store selection metadata.
- GET /history?session_id=: return messages + citations + selections.

## Data Pipeline
- Chunk book by structure/length; store metadata {chapter, section, page, uri, chunk_id}.
- Embed chunks with OpenAI embeddings; upsert to Qdrant with payload metadata.
- Maintain overlap for context; size chunks for latency and recall balance.

## Orchestration
- Use OpenAI Agents/ChatKit (model: 4o mini) to build prompts and stream responses.
- Inject retrieved chunks (or selection subset) plus system/assistant prompts.
- Enforce citation requirement and selection-only policy in prompt.

## Persistence
- Neon Postgres tables: sessions, messages, selections, citations.
- Store retrieval metadata (scores, chunk_ids) for audit/debug.

## Constraints
- No external knowledge; answers must be grounded in book text.
- Selection-only: if selection subset lacks support, respond “insufficient evidence from the selection.”
- Non-selection: if top-k lacks support, refuse/ask for clarification.

## Risk Management
- Strict grounding and refusal on insufficient evidence.
- Rate-limit per user/session/IP to protect stability.
- Minimize logged data; protect user selections and chat history.
- Graceful degradation if Qdrant/Neon unavailable; surface clear errors.
- Monitor latency; target end-to-end <~5s typical.
