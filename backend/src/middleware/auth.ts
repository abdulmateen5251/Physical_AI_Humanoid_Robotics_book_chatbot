/**
 * Authentication Middleware
 * 
 * Provides session verification and route protection.
 * 
 * Task: T043
 * 
 * Middleware:
 * - requireAuth: Ensures user is authenticated
 * - optionalAuth: Attaches user session if available (for personalization)
 */

import { Request, Response, NextFunction } from 'express';

// TODO: Task T043 - Create authentication middleware
// Implementation: M1 - Week 2-3

/**
 * Require authentication middleware
 * 
 * Verifies session exists and is valid.
 * Returns 401 Unauthorized if not authenticated.
 */
export const requireAuth = async (
  req: Request,
  res: Response,
  next: NextFunction
): Promise<void> => {
  // Implementation: M1 - Week 2-3
  // - Session verification
  // - Attach user to request object
  // - Route protection
  
  // Placeholder: Allow all requests during development
  if (process.env.NODE_ENV === 'development') {
    return next();
  }
  
  res.status(401).json({ error: 'Authentication required' });
};

/**
 * Optional authentication middleware
 * 
 * Attaches user session if available, but doesn't require it.
 * Used for endpoints that personalize based on auth status.
 */
export const optionalAuth = async (
  req: Request,
  _res: Response,
  next: NextFunction
): Promise<void> => {
  // Implementation: M1 - Week 2-3
  // - Check for session
  // - Attach user if found
  // - Continue regardless of auth status
  
  // Placeholder: Continue without auth
  next();
};
