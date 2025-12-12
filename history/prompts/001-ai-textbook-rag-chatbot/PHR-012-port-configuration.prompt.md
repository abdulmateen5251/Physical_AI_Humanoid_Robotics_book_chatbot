# PHR-012: Port Configuration Update (8000 → 8090)

**Date**: December 10, 2025  
**Phase**: Infrastructure Adjustment  
**Status**: ✅ Complete

## Context

User requested to change the backend port from 8000 to 8090 to avoid conflicts with other services.

## User Request

> "change the port from 8000 to 8090 everywhere where it is using"

## Implementation Summary

### Files Modified (15+ files)

1. **Docker Configuration**
   - docker-compose.yml: Changed port mapping to `8090:8000`

2. **Frontend Files**
   - src/utils/apiClient.js: `API_BASE_URL = 'http://localhost:8090'`
   - src/components/ChatWidget.tsx: Changed fetch URL to `http://localhost:8090/chat`

3. **Documentation Files**
   - README.md
   - SETUP_GUIDE.md
   - DOCKER_SETUP.md
   - RAG_SETUP_GUIDE.md
   - IMPLEMENTATION_COMPLETE.md
   - And 10+ other documentation files

### Port Mapping

```
Before: localhost:8000 → container:8000
After:  localhost:8090 → container:8000
```

Internal container port remains 8000, external access via 8090.

## Technical Details

### Docker Compose Change
```yaml
backend:
  ports:
    - "8090:8000"  # Changed from "8000:8000"
  healthcheck:
    test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
    # Note: Health check uses internal port (8000)
```

### Frontend API Client Change
```javascript
// Before
const API_BASE_URL = 'http://localhost:8000/api';

// After
const API_BASE_URL = 'http://localhost:8090';
```

### ChatWidget Change
```typescript
// Before
fetch('http://localhost:8001/chat', ...)

// After
fetch('http://localhost:8090/chat', ...)
```

## Commands

```bash
# Rebuild with new port
docker compose down
docker compose up -d --build

# Test new port
curl http://localhost:8090/health
```

## Issues Encountered

**Issue**: ChatWidget was using hardcoded URL `localhost:8001`
**Solution**: Updated to `localhost:8090` to match backend port

## Outcome

✅ All references updated to port 8090
✅ Frontend-backend communication working
✅ Health checks operational
✅ Documentation updated
✅ No port conflicts

## Files Changed Summary

- 1 Docker Compose file
- 2 Frontend source files
- 12+ documentation files
- Total: 15+ files updated
